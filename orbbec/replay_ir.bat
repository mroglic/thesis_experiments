@echo off
rem Plays the newest IR recording (or the folder given) through Spectacular AI with the 3D view
rem e.g.  replay_ir.bat   or   replay_ir.bat recordings\2026-10-29_12-32-02   or   replay_ir.bat recordings\ir_x --no_slam
set PATH=%LOCALAPPDATA%\Microsoft\WinGet\Links;%LOCALAPPDATA%\Programs\Python\Python312;%PATH%
cd /d "%~dp0"
set FOLDER=%1
if "%FOLDER%"=="" for /f "delims=" %%d in ('dir /b /ad /o-d recordings\ir_*') do if not defined FOLDER set FOLDER=recordings\%%d
python ir_replay.py %FOLDER% %2 %3 %4 %5
pause
