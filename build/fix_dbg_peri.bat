@echo off
rem ---------------------------------------------------------------
rem  Fix "Project name doesn't match with design file" (Debugger Step 3).
rem  A stale ti60f225_oob.dbg.peri.xml left in the project root by an
rem  aborted run shadows ti60f225_oob.peri.xml in efx_pt_jtag_util.py's
rem  directory scan (it checks only the FIRST *.peri.xml found, and
rem  "ti60f225_oob.dbg." sorts before "ti60f225_oob.").
rem  Efinity normally keeps this file in outflow\ - move it back there,
rem  then re-run the compile flow.
rem  usage:  build\fix_dbg_peri.bat
rem ---------------------------------------------------------------
setlocal
set ROOT=%~dp0..
if exist "%ROOT%\ti60f225_oob.dbg.peri.xml" (
    if not exist "%ROOT%\outflow" mkdir "%ROOT%\outflow"
    move /y "%ROOT%\ti60f225_oob.dbg.peri.xml" "%ROOT%\outflow\ti60f225_oob.dbg.peri.xml" >nul
    echo [OK] stale ti60f225_oob.dbg.peri.xml moved to outflow\ - re-run the compile flow now.
) else (
    echo [OK] no stale dbg.peri.xml in project root - nothing to fix.
)
endlocal
