@echo off
rem Light test "dark_flood": one run of ir_vio.py --test dark_flood, logged to runs\dark_flood_<n>.jsonl (see README)
rem Extra options are passed on, e.g.  dark_flood.bat --ir_pos 1940,40
cd /d "%~dp0.."
if exist .venv\Scripts\python.exe (set PY=.venv\Scripts\python) else (set PY=python)
%PY% oak\ir_vio.py --test dark_flood %*
%PY% oak\compare_runs.py
pause
