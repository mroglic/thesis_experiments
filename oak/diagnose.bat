@echo off
rem HTML report of a Spectacular AI recording (sai-cli diagnose): IMU and frame rates, gaps, timestamps.
rem   diagnose.bat                              newest recordings\sai\*
rem   diagnose.bat recordings\sai\<folder>
pushd "%~dp0.."
if exist .venv\Scripts\python.exe (set PY=.venv\Scripts\python) else (set PY=python)
set FOLDER=%1
if "%FOLDER%"=="" for /f "delims=" %%d in ('dir /b /ad /o-d recordings\sai') do if not defined FOLDER set FOLDER=recordings\sai\%%d
if "%FOLDER%"=="" (echo No recording in recordings\sai.& popd & pause & exit /b)
echo %FOLDER%
dir "%FOLDER%"
%PY% -m spectacularAI.cli.sai_cli diagnose "%FOLDER%" "%FOLDER%\report.html" && start "" "%FOLDER%\report.html"
pause
popd
