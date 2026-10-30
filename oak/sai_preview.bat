@echo off
rem Spectacular AI's own OAK preview (sai-cli record oak --preview3d), without our script: if tracking is lost here
rem too, the cause is the camera, cable or SDK, not ir_vio.py. No SLAM, no RGB, like the tests; no IR light.
rem It always records, to recordings\sai\<time> (delete when done). Extra options are passed on, e.g.
rem   sai_preview.bat --no_feature_tracker     (feature tracking on the PC instead of the OAK)
rem   sai_preview.bat --gray_preview           (also show the gray image)
pushd "%~dp0.."
if exist .venv\Scripts\python.exe (set PY=.venv\Scripts\python) else (set PY=python)
%PY% -m spectacularAI.cli.sai_cli record oak --preview3d --no_slam --no_rgb --output recordings\sai --auto_subfolders %*
pause
popd
