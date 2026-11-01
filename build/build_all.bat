@echo off
rem ---------------------------------------------------------------
rem  new_project : full compile flow (synthesis + PnR + bitstream)
rem  usage:  build\build_all.bat
rem ---------------------------------------------------------------
setlocal
if not defined EFINITY_HOME set "EFINITY_HOME=C:\Efinity\2026.1"
call "%EFINITY_HOME%\bin\setup.bat"
cd /d "%~dp0.."
rem guard: Efinity GUI may rewrite the project XML and drop the sensor crop stage,
rem which changes Bayer row phase and causes a magenta/noisy picture.
findstr /L /C:"rtl/sensor_clipper.v" "ti60f225_oob.xml" >nul
if errorlevel 1 (
    echo [ERROR] rtl/sensor_clipper.v is missing from ti60f225_oob.xml
    echo [ERROR] Re-add it under Sources before compiling, or restore the XML from its backup.
    exit /b 1
)
findstr /L /C:"rtl/cam_i2c_ctrl/i2c/cam_cfg_sched.v" "ti60f225_oob.xml" >nul
if errorlevel 1 (
    echo [ERROR] rtl/cam_i2c_ctrl/i2c/cam_cfg_sched.v is missing from ti60f225_oob.xml
    echo [ERROR] Re-add it under Sources before compiling, or restore the XML from its backup.
    exit /b 1
)
rem guard: stale dbg.peri.xml in the root breaks Debugger Step 3 on the next run
call "%~dp0fix_dbg_peri.bat" >nul
echo [INFO] project dir: %CD%
call efx_run.bat --prj -f compile ti60f225_oob.xml
if errorlevel 1 (
    echo [ERROR] compile failed - see outflow\*.log
    exit /b 1
)
echo [INFO] compile done.
echo [INFO] bitstream : %CD%\outflow\ti60f225_oob.bit
endlocal
