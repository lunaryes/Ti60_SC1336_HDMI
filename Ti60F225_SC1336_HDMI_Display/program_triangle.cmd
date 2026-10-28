@echo off
setlocal
set "TEMP=F:\codex\tmp"
set "TMP=F:\codex\tmp"
cd /d "%~dp0"
if not exist "outflow_triangle\Ti60_Demo.bit" exit /b 1
call "E:\qianrushi\bin\setup.bat"
if errorlevel 1 exit /b 1
call "E:\qianrushi\pgm\bin\ftdi_pgm.bat" --board_profile "Generic Board Profile Using FT232H" --mode jtag --url ftdi://0x0403:0x6014:0:ff/1 --aurl ftdi://0x0403:0x6014:0:ff/1 --jtag_clock_freq 6000000 outflow_triangle\Ti60_Demo.bit
exit /b %errorlevel%
