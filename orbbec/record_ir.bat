@echo off
rem Records Femto Mega ToF IR + depth + IMU for Spectacular AI (preview window, q stops) into recordings\ir_<time>
rem Options are passed on, e.g.  record_ir.bat --resolution 640x576   or   --ir_max 800   or   --no_depth
set PATH=%LOCALAPPDATA%\Microsoft\WinGet\Links;%LOCALAPPDATA%\Programs\Python\Python312;%PATH%
cd /d "%~dp0"
python ir_record.py %*
pause
