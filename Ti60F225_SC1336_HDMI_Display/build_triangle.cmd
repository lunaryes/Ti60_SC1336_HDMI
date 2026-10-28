@echo off
setlocal
set "TEMP=F:\codex\tmp"
set "TMP=F:\codex\tmp"
cd /d "%~dp0"
if not exist "E:\qianrushi\bin\setup.bat" (
  echo Efinity setup not found: E:\qianrushi\bin\setup.bat
  exit /b 1
)
call "E:\qianrushi\bin\setup.bat"
if errorlevel 1 exit /b 1
call "E:\qianrushi\bin\efx_run.bat" Ti60_Demo.xml --flow compile --output_dir outflow_triangle --work_dir work_triangle
exit /b %errorlevel%
