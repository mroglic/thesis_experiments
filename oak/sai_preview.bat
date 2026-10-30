@echo off
rem Spectacular AI's own OAK preview (sai-cli record oak --preview3d), without our script: if tracking is lost here
rem too, the cause is the camera, cable or SDK, not ir_vio.py. With SLAM (point cloud) like ir_vio.py, gray image window, no RGB, no IR light.
rem It always records, to recordings\sai\<time> (delete when done). Extra options are passed on, e.g.
rem   sai_preview.bat --no_feature_tracker     (feature tracking on the PC instead of the OAK)
rem   sai_preview.bat --no_slam                (VIO only, like the tests: no point cloud, only the camera and its trail)
pushd "%~dp0.."
if exist .venv\Scripts\python.exe (set PY=.venv\Scripts\python) else (set PY=python)
%PY% -m spectacularAI.cli.sai_cli record oak --preview3d --gray_preview --no_rgb --output recordings\sai --auto_subfolders %*
pause
popd
