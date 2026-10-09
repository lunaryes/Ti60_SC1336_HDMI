@echo off
REM ===================================================================
REM  编译脚本（已修正过期路径）
REM  原脚本用的 c:\efinity\2022.2 与工程 sw_version="2026.1.132" 不匹配，
REM  且 wsl 在本机被安全策略拦截。
REM
REM  本机匹配的工具链：
REM    E:\qianrushi\efinity-2026.1.132-windows-x64\efinity\2026.1\bin
REM  用法：在 Ti60F225_SC1336_HDMI_Display 目录下执行 start.cmd
REM ===================================================================

set EFINITY_BIN=E:\qianrushi\efinity-2026.1.132-windows-x64\efinity\2026.1\bin

if exist outflow  rmdir /s /q outflow
if exist work_syn rmdir /s /q work_syn
if exist work_pnr rmdir /s /q work_pnr
if exist work_pt  rmdir /s /q work_pt

"%EFINITY_BIN%\efx_run.bat" Ti60_Demo.xml

pause
