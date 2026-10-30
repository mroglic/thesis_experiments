@echo off
rem Deletes all light test runs (runs\*.jsonl), after asking
pushd "%~dp0.."
if not exist runs\*.jsonl (echo No runs to delete.& popd & pause & exit /b)
dir /b runs\*.jsonl
choice /m "Delete these runs permanently"
if errorlevel 2 (echo Nothing deleted.) else (del /q runs\*.jsonl & echo Deleted.)
pause
popd
