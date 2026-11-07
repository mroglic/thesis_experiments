#!/bin/bash
# ORB-SLAM3 Stereo-Inertial on an OAK recording (Mac/Linux): converts it to EuRoC (sai_to_euroc.py), runs
# stereo_inertial_euroc, then prints the summary (orb_eval.py). Extra options go to sai_to_euroc.py, e.g.
#   oak/run_orbslam3.sh recordings/dark_flood_1
#   oak/run_orbslam3.sh recordings/dark_flood_1 --mask --force       (projector rectangle blacked out, reconvert)
#   oak/run_orbslam3.sh recordings/dark_1 --ini_fast 10 --min_fast 4 --force   (more corners in dark images)
# ORB-SLAM3 location: $ORB_SLAM3 (default ~/dev/slam/ORB_SLAM3, built with Examples/Stereo-Inertial).
set -e
cd "$(dirname "$0")/.."
REC="${1:?usage: oak/run_orbslam3.sh <recording folder> [sai_to_euroc.py options]}"; shift
ORB="${ORB_SLAM3:-$HOME/dev/slam/ORB_SLAM3}"
BIN="$ORB/Examples/Stereo-Inertial/stereo_inertial_euroc"
[ -x "$BIN" ] || { echo "no $BIN (build ORB-SLAM3 first)"; exit 1; }
if [ -x .venv/bin/python ]; then PY=.venv/bin/python; else PY=python3; fi
$PY -c "import cv2, numpy" 2>/dev/null || { echo "$PY has no opencv/numpy: uv venv .venv && uv pip install --python .venv/bin/python numpy opencv-python"; exit 1; }

OUT="$REC/euroc"
if [ ! -f "$OUT/orbslam3.yaml" ] || [[ " $* " == *" --force "* ]]; then
  $PY oak/sai_to_euroc.py "$REC" "$@"
fi
cd "$OUT"
rm -f f_orb.txt kf_orb.txt
"$BIN" "$ORB/Vocabulary/ORBvoc.txt" orbslam3.yaml . times.txt orb 2>&1 | tee orbslam3.log
cd - > /dev/null
$PY oak/orb_eval.py "$OUT"
