@echo off
rem Light test "light": one run of ir_vio.py --test light, logged to runs\light_<n>.jsonl (see README)
rem Extra options are passed on, e.g.  light.bat --ir_pos 1940,40
pushd "%~dp0.."
if exist .venv\Scripts\python.exe (set PY=.venv\Scripts\python) else (set PY=python)
%PY% oak\ir_vio.py --test light %*
%PY% oak\compare_runs.py
pause
popd
