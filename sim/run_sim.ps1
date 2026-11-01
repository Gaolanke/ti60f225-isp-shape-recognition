[CmdletBinding()]
param(
    [ValidateSet('edge', 'sobel', 'chain', 'scale', 'roi', 'gamma', 'debayer', 'clipper', 'sensor', 'camcfg', 'awb', 'shape', 'all', 'clean', 'wave')]
    [string]$Test = 'all',
    [switch]$Strict
)

$ErrorActionPreference = 'Stop'

$simRoot = $PSScriptRoot
$projRoot = Split-Path -Parent $simRoot
$outDir = Join-Path $simRoot 'out'
$waveDir = Join-Path $simRoot 'wave'
$tbDir = Join-Path $simRoot 'tb'
$edgeRtl = Join-Path $projRoot 'rtl\edge\edge_proc.v'
$ppmToPng = Join-Path $simRoot 'tools\ppm_to_png.py'

function Find-Executable {
    param(
        [string]$Name,
        [string[]]$Candidates
    )
    foreach ($candidate in $Candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate)) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    return $null
}

$iverilogCandidates = @(
    $env:IVERILOG,
    (Join-Path $projRoot 'tools\iverilog\bin\iverilog.exe'),
    (Join-Path $simRoot 'tools\iverilog\bin\iverilog.exe'),
    (Join-Path $env:USERPROFILE 'Downloads\Ti60F225_DemoBoard_v4\tools\iverilog\bin\iverilog.exe')
)
$iverilog = Find-Executable -Name 'iverilog.exe' -Candidates $iverilogCandidates
if (-not $iverilog) {
    throw 'iverilog.exe not found. Set IVERILOG or put Icarus Verilog under tools\iverilog\bin.'
}
$binDir = Split-Path -Parent $iverilog
$vvp = Find-Executable -Name 'vvp.exe' -Candidates @((Join-Path $binDir 'vvp.exe'))
if (-not $vvp) { throw 'vvp.exe not found next to iverilog.exe.' }

$gtkwave = Find-Executable -Name 'gtkwave.exe' -Candidates @(
    (Join-Path $binDir 'gtkwave.exe'),
    (Join-Path $projRoot 'tools\iverilog\gtkwave\bin\gtkwave.exe'),
    (Join-Path $simRoot 'tools\iverilog\gtkwave\bin\gtkwave.exe'),
    (Join-Path $env:USERPROFILE 'Downloads\Ti60F225_DemoBoard_v4\tools\iverilog\gtkwave\bin\gtkwave.exe')
)

function Ensure-Dirs {
    New-Item -ItemType Directory -Force -Path $outDir, $waveDir | Out-Null
}

function Remove-GeneratedFiles {
    foreach ($dir in @($outDir, $waveDir)) {
        if (-not (Test-Path -LiteralPath $dir)) { continue }
        Get-ChildItem -LiteralPath $dir -File |
            Where-Object { $_.Extension -in @('.vvp', '.vcd', '.ppm', '.png', '.log') } |
            ForEach-Object { Remove-Item -LiteralPath $_.FullName -Force }
    }
    Write-Host 'Removed generated simulation files.'
}

function Convert-PpmOutputs {
    $python = Find-Executable -Name 'python.exe' -Candidates @()
    if (-not $python) {
        $python = Find-Executable -Name 'python' -Candidates @()
    }
    if (-not $python) {
        Write-Warning 'Python not found; PPM files are available but PNG conversion was skipped.'
        return
    }
    Get-ChildItem -LiteralPath $outDir -File -Filter '*.ppm' | ForEach-Object {
        & $python $ppmToPng $_.FullName --scale 4
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "PNG conversion failed for $($_.Name)"
        }
    }
}

function Invoke-HdlTest {
    param(
        [string]$Name,
        [string]$Top,
        [string]$Testbench,
        [string[]]$ExtraRtl = @(),
        [switch]$Vcd
    )
    $vvpPath = Join-Path $outDir "$Name.vvp"
    $logPath = Join-Path $outDir "$Name.log"
    $compileArgs = @('-g2012', '-o', $vvpPath, '-s', $Top, $edgeRtl) + $ExtraRtl + @($Testbench)
    Write-Host "Compiling $Name ..."
    & $iverilog @compileArgs
    if ($LASTEXITCODE -ne 0) { throw "$Name compile failed." }

    $runArgs = @($vvpPath)
    if ($Vcd) { $runArgs += '+vcd' }
    if ($Strict) { $runArgs += '+strict' }
    Write-Host "Running $Name ..."
    $lines = & $vvp @runArgs 2>&1
    $lines | Tee-Object -FilePath $logPath | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "$Name simulation failed." }
    Convert-PpmOutputs
}

Ensure-Dirs

switch ($Test) {
    'clean' {
        Remove-GeneratedFiles
    }
    'edge' {
        Invoke-HdlTest -Name 'tb_edge_proc' -Top 'tb_edge_proc' `
            -Testbench (Join-Path $tbDir 'tb_edge_proc.v')
    }
    'sobel' {
        Invoke-HdlTest -Name 'tb_sobel_mode' -Top 'tb_sobel_mode' `
            -Testbench (Join-Path $tbDir 'tb_sobel_mode.v')
    }
    'chain' {
        Invoke-HdlTest -Name 'tb_edge_chain' -Top 'tb_edge_chain' `
            -Testbench (Join-Path $tbDir 'tb_edge_chain.v')
    }
    'scale' {
        Invoke-HdlTest -Name 'tb_shape_scale' -Top 'tb_shape_scale' `
            -Testbench (Join-Path $tbDir 'tb_shape_scale.v')
    }
    'roi' {
        Invoke-HdlTest -Name 'tb_shape_roi' -Top 'tb_shape_roi' `
            -Testbench (Join-Path $tbDir 'tb_shape_roi.v')
    }
    'gamma' {
        Invoke-HdlTest -Name 'tb_gamma_correction' -Top 'tb_gamma_correction' `
            -Testbench (Join-Path $tbDir 'tb_gamma_correction.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\gamma_conrrection\gamma_correction.v')
    }
    'debayer' {
        Invoke-HdlTest -Name 'tb_debayer_bg' -Top 'tb_debayer_bg' `
            -Testbench (Join-Path $tbDir 'tb_debayer_bg.v') `
            -ExtraRtl @(
                (Join-Path $projRoot 'rtl\debayer\true_dual_port_ram.v'),
                (Join-Path $projRoot 'rtl\debayer\line_buffer.v'),
                (Join-Path $projRoot 'rtl\debayer\rgb_gain_v1.v'),
                (Join-Path $projRoot 'rtl\debayer\raw_to_rgb.v'),
                (Join-Path $projRoot 'rtl\debayer\debayer_top_2to1.v')
            )
    }
    'clipper' {
        Invoke-HdlTest -Name 'tb_sensor_clipper' -Top 'tb_sensor_clipper' `
            -Testbench (Join-Path $tbDir 'tb_sensor_clipper.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\sensor_clipper.v')
    }
    'sensor' {
        Invoke-HdlTest -Name 'tb_sensor_gain' -Top 'tb_sensor_gain' `
            -Testbench (Join-Path $tbDir 'tb_sensor_gain.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\cam_i2c_ctrl\i2c\i2c_master_reg_rom.v')
    }
    'awb' {
        Invoke-HdlTest -Name 'tb_cam_ae_awb' -Top 'tb_cam_ae_awb' `
            -Testbench (Join-Path $tbDir 'tb_cam_ae_awb.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\cam_ae_awb.v')
    }
    'shape' {
        Invoke-HdlTest -Name 'tb_shape_rec' -Top 'tb_shape_rec' `
            -Testbench (Join-Path $tbDir 'tb_shape_rec.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\edge\shape_rec.v')
    }
    'camcfg' {
        Invoke-HdlTest -Name 'tb_cam_cfg_sched' -Top 'tb_cam_cfg_sched' `
            -Testbench (Join-Path $tbDir 'tb_cam_cfg_sched.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\cam_i2c_ctrl\i2c\cam_cfg_sched.v')
    }
    'all' {
        Invoke-HdlTest -Name 'tb_sensor_clipper' -Top 'tb_sensor_clipper' `
            -Testbench (Join-Path $tbDir 'tb_sensor_clipper.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\sensor_clipper.v')
        Invoke-HdlTest -Name 'tb_edge_proc' -Top 'tb_edge_proc' `
            -Testbench (Join-Path $tbDir 'tb_edge_proc.v')
        Invoke-HdlTest -Name 'tb_sobel_mode' -Top 'tb_sobel_mode' `
            -Testbench (Join-Path $tbDir 'tb_sobel_mode.v')
        Invoke-HdlTest -Name 'tb_edge_chain' -Top 'tb_edge_chain' `
            -Testbench (Join-Path $tbDir 'tb_edge_chain.v')
        Invoke-HdlTest -Name 'tb_gamma_correction' -Top 'tb_gamma_correction' `
            -Testbench (Join-Path $tbDir 'tb_gamma_correction.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\gamma_conrrection\gamma_correction.v')
        Invoke-HdlTest -Name 'tb_debayer_bg' -Top 'tb_debayer_bg' `
            -Testbench (Join-Path $tbDir 'tb_debayer_bg.v') `
            -ExtraRtl @(
                (Join-Path $projRoot 'rtl\debayer\true_dual_port_ram.v'),
                (Join-Path $projRoot 'rtl\debayer\line_buffer.v'),
                (Join-Path $projRoot 'rtl\debayer\rgb_gain_v1.v'),
                (Join-Path $projRoot 'rtl\debayer\raw_to_rgb.v'),
                (Join-Path $projRoot 'rtl\debayer\debayer_top_2to1.v')
            )
        Invoke-HdlTest -Name 'tb_sensor_gain' -Top 'tb_sensor_gain' `
            -Testbench (Join-Path $tbDir 'tb_sensor_gain.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\cam_i2c_ctrl\i2c\i2c_master_reg_rom.v')
        Invoke-HdlTest -Name 'tb_cam_cfg_sched' -Top 'tb_cam_cfg_sched' `
            -Testbench (Join-Path $tbDir 'tb_cam_cfg_sched.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\cam_i2c_ctrl\i2c\cam_cfg_sched.v')
        Invoke-HdlTest -Name 'tb_cam_ae_awb' -Top 'tb_cam_ae_awb' `
            -Testbench (Join-Path $tbDir 'tb_cam_ae_awb.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\cam_ae_awb.v')
        Invoke-HdlTest -Name 'tb_shape_rec' -Top 'tb_shape_rec' `
            -Testbench (Join-Path $tbDir 'tb_shape_rec.v') `
            -ExtraRtl (Join-Path $projRoot 'rtl\edge\shape_rec.v')
    }
    'wave' {
        Invoke-HdlTest -Name 'tb_edge_proc' -Top 'tb_edge_proc' `
            -Testbench (Join-Path $tbDir 'tb_edge_proc.v') -Vcd
        if ($gtkwave) {
            Start-Process -FilePath $gtkwave -ArgumentList @((Join-Path $waveDir 'tb_edge_proc.vcd'))
        } else {
            Write-Warning "GTKWave not found; waveform is in $waveDir\tb_edge_proc.vcd"
        }
    }
}

Write-Host "Done. Results: $outDir"
