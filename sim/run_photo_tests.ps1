[CmdletBinding()]
param(
    [int]$Frames = 16
)

$ErrorActionPreference = 'Stop'

$simRoot   = $PSScriptRoot
$projRoot  = Split-Path -Parent $simRoot
$outDir    = Join-Path $simRoot 'out'
$genDir    = Join-Path $simRoot 'images\gen'
$edgeRtl   = Join-Path $projRoot 'rtl\edge\edge_proc.v'
$testbench = Join-Path $simRoot 'tb\tb_shape_roi.v'

function Find-Executable {
    param([string]$Name, [string[]]$Candidates)
    foreach ($candidate in $Candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate)) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    return $null
}

$iverilog = Find-Executable -Name 'iverilog.exe' -Candidates @(
    $env:IVERILOG,
    (Join-Path $projRoot 'tools\iverilog\bin\iverilog.exe'),
    (Join-Path $simRoot 'tools\iverilog\bin\iverilog.exe'),
    (Join-Path $env:USERPROFILE 'Downloads\Ti60F225_DemoBoard_v4\tools\iverilog\bin\iverilog.exe')
)
if (-not $iverilog) { throw 'iverilog.exe not found. Set IVERILOG or install Icarus Verilog.' }
$vvp = Find-Executable -Name 'vvp.exe' -Candidates @((Join-Path (Split-Path -Parent $iverilog) 'vvp.exe'))
if (-not $vvp) { throw 'vvp.exe not found next to iverilog.exe.' }

New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$vvpFile = Join-Path $outDir 'tb_shape_photo.vvp'

& $iverilog -g2012 -o $vvpFile -s tb_shape_roi $edgeRtl $testbench
if ($LASTEXITCODE -ne 0) { throw 'tb_shape_roi compile failed.' }

$cases = @(
    @{ Name = 'paper_target';      Expect = 7; Frames = $Frames },
    @{ Name = 'paper_target_tall'; Expect = 2; Frames = ($Frames - 2) }
)

$failures = 0
foreach ($case in $cases) {
    $prefix = Join-Path $genDir $case.Name
    $log = Join-Path $outDir ('photo_' + $case.Name + '.log')
    $imgArg = '+imgfile=' + $prefix
    $frameArg = '+img_frames=' + $case.Frames
    $expectArg = '+expect=' + $case.Expect
    Write-Host ('Running ' + $case.Name + ' ...')
    & $vvp $vvpFile +imgtest $imgArg +img_asp=4 $frameArg $expectArg 2>&1 |
        Tee-Object -FilePath $log
    $line = Select-String -Path $log -Pattern '^RESULT:' | Select-Object -Last 1
    if (($LASTEXITCODE -ne 0) -or (-not $line) -or ($line.Line -notmatch 'PASS')) {
        Write-Host ('FAILED: ' + $case.Name)
        $failures++
    }
}

if ($failures -ne 0) {
    throw ($failures.ToString() + ' uploaded-photo test(s) failed.')
}
Write-Host 'Uploaded-photo target tests PASS.'
