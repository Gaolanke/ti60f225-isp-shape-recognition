@echo off
rem ---------------------------------------------------------------
rem  download bitstream to Ti60F225I3 board through FTDI JTAG
rem  usage:  build\flash_jtag.bat          (board powered + USB connected)
rem ---------------------------------------------------------------
setlocal
if not defined EFINITY_HOME set "EFINITY_HOME=C:\Efinity\2026.1"
call "%EFINITY_HOME%\bin\setup.bat"
cd /d "%~dp0.."
set EFX_HIFTDI_ENABLE=1
if "%~1"=="" (
    call ftdi_pgm.bat "outflow\ti60f225_oob.bit" -m jtag
) else (
    call ftdi_pgm.bat "outflow\ti60f225_oob.bit" -m jtag -u %1
)
endlocal
