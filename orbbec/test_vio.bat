@echo off
rem Smoke test: VIO poses as JSON lines in the console (Ctrl+C to stop)
cd /d "%~dp0Windows\bin"
vio_jsonl.exe
pause
