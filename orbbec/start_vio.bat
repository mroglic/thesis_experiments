@echo off
rem Spectacular AI VIO on the Orbbec Femto Mega: live 3D view + recording to sai\recordings\<time>
rem Extra options are passed on, e.g.  start_vio.bat --no_record   or   --recording_only --no_preview
set PATH=%LOCALAPPDATA%\Microsoft\WinGet\Links;%LOCALAPPDATA%\Programs\Python\Python312;%LOCALAPPDATA%\Programs\Python\Python312\Scripts;%PATH%
cd /d "%~dp0Windows\bin"
sai-record-orbbec.exe --output "%~dp0recordings" --auto_subfolders %*
pause
