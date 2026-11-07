r"""
Summary of an ORB-SLAM3 run on a converted recording (run_orbslam3.sh), in the same terms as compare_runs.py:

    python orb_eval.py recordings/dark_flood_1/euroc [more euroc folders ...]

From <euroc>/f_orb.txt (frame trajectory of the biggest map, EuRoC format: t[ns] tx ty tz qx qy qz qw),
<euroc>/times.txt (all frames) and <euroc>/orbslam3.log (console output):
    frames     stereo pairs given to ORB-SLAM3
    in map     share of frames that ended up in the biggest map (frames before IMU init or in other maps are missing)
    maps       maps ORB-SLAM3 started (1 = never lost; each loss without relocalisation starts a new map)
    lost       "LOST" / "Fail to track local map" messages in the log
    path       path length of the trajectory [m]
    end-start  distance between the last and the first pose [m] (drift, if the route ends where it started)
Note: ORB-SLAM3 runs with loop closing, SAI's light tests run without SLAM; a loop closure at the mark can hide drift.
"""
import math
import os
import re
import sys


def summary(folder):
    traj = os.path.join(folder, "f_orb.txt")
    if not os.path.exists(traj):
        return {"run": folder, "error": "no f_orb.txt (did ORB-SLAM3 finish?)"}
    with open(os.path.join(folder, "times.txt")) as f:
        n_frames = sum(1 for line in f if line.strip())
    poses = []
    with open(traj) as f:
        for line in f:
            v = line.split()
            if len(v) >= 4:
                poses.append((float(v[0]) / 1e9, [float(x) for x in v[1:4]]))
    log = ""
    if os.path.exists(os.path.join(folder, "orbslam3.log")):
        with open(os.path.join(folder, "orbslam3.log"), errors="replace") as f:
            log = f.read()
    maps = 1 + len(re.findall(r"Creation of new map", log))
    lost = len(re.findall(r"LOST|Fail to track local map", log))
    path = sum(math.dist(a[1], b[1]) for a, b in zip(poses, poses[1:]))
    end_start = math.dist(poses[0][1], poses[-1][1]) if poses else float("nan")
    dur = poses[-1][0] - poses[0][0] if len(poses) > 1 else 0.0
    return {"run": os.path.basename(os.path.dirname(os.path.abspath(folder))) or folder, "frames": n_frames,
            "in map": f"{100 * len(poses) / max(n_frames, 1):.0f} %", "maps": maps, "lost": lost,
            "duration": f"{dur:.0f} s", "path": f"{path:.2f} m", "end-start": f"{end_start:.3f} m"}


def main():
    folders = sys.argv[1:]
    if not folders:
        raise SystemExit(__doc__)
    rows = [summary(f) for f in folders]
    cols = ["run", "frames", "in map", "maps", "lost", "duration", "path", "end-start", "error"]
    cols = [c for c in cols if any(c in r for r in rows)]
    width = {c: max(len(c), *(len(str(r.get(c, ""))) for r in rows)) for c in cols}
    print("  ".join(c.ljust(width[c]) for c in cols))
    for r in rows:
        print("  ".join(str(r.get(c, "")).ljust(width[c]) for c in cols))


if __name__ == "__main__":
    main()
