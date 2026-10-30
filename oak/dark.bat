@echo off
rem Light test "dark": one run of ir_vio.py --test dark, logged to runs\dark_<n>.jsonl (see README)
rem Extra options are passed on, e.g.  dark.bat --ir_pos 1940,40
pushd "%~dp0.."
if exist .venv\Scripts\python.exe (set PY=.venv\Scripts\python) else (set PY=python)
%PY% oak\ir_vio.py --test dark %*
%PY% oak\compare_runs.py
pause
popd
