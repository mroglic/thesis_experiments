r"""
Spectacular AI OAK recording -> EuRoC dataset + ORB-SLAM3 settings, so ORB-SLAM3 (Stereo-Inertial) runs on exactly
the input SAI got: the rectified left/right mono video (OV9282, the IR view) and the IMU.

    python sai_to_euroc.py recordings/dark_flood_1                 # -> recordings/dark_flood_1/euroc/
    python sai_to_euroc.py recordings/dark_flood_1 --mask          # black out ARML's projector rectangle
    python sai_to_euroc.py recordings/sai/<time> --out /tmp/x      # a sai_preview.bat / sai-cli --gray recording

Input (from `ir_vio.py --record` or `sai-cli record oak --gray`):
    data.jsonl                      IMU (gyroscope, accelerometer) and the VIO frame times
    calibration.json                the two (rectified) cameras: intrinsics, imuToCamera
    rectified_left/right.mp4|.h264  the video
    rectified_<eye>_frames.csv      sequence number + device time per video frame (ir_vio.py only)

Output (<out>/):
    mav0/cam0/data/<ns>.png, mav0/cam1/data/<ns>.png    left / right, same name = same time
    mav0/imu0/data.csv              t [ns], gyro xyz [rad/s], acc xyz [m/s^2] (acc interpolated to the gyro times)
    times.txt                       one timestamp [ns] per stereo pair (ORB-SLAM3's times file)
    orbslam3.yaml                   two identical pinholes without distortion + Stereo.T_c1_c2, IMU.T_b_c1, IMU noise
                                    (not Camera.type "Rectified": ORB-SLAM3 1.0 crashes with it in stereo, it never
                                    loads camera 2; re-rectifying already rectified images with R = I is a no-op)
    convert.json                    what was found (frame counts, clock offset, gaps), for the thesis log

Times: with the CSV every frame gets its own device time; the offset to data.jsonl's clock is measured on the
frames both have (nearest match) and removed, so images and IMU share one clock. Without the CSV the video frames
are assumed to be data.jsonl's frames in order (as SAI's own Replay assumes); the counts are checked.

--mask blacks out the same normalised rectangle as ir_vio.py --mask (x0,y0,x1,y1, ARML's trackerMasks by default) on
both images, so ORB-SLAM3 also finds no features where the projector shines. On the right image the projector
area sits a little further left (disparity); the same rectangle is used on both, as SAI does.

IMU noise: defaults are the BMI270 datasheet densities x10 (consumer IMU, ORB-SLAM3 needs headroom); change them
with --noise_gyro etc. Needs: numpy, opencv-python.
"""
import argparse
import bisect
import csv
import glob
import json
import math
import os
import shutil
import statistics
import sys

ARML_MASK = "0.314,0.347,0.700,0.748"


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("recording", help="recording folder (data.jsonl, calibration.json, rectified_*.mp4/.h264)")
    p.add_argument("--out", help="output folder (default <recording>/euroc)")
    p.add_argument("--mask", nargs="?", const=ARML_MASK, default=None, metavar="X0,Y0,X1,Y1",
                   help=f"black out this normalised rectangle on both images (default ARML's {ARML_MASK})")
    p.add_argument("--noise_gyro", type=float, default=1.2e-3, help="gyro noise density [rad/s/sqrt(Hz)] (BMI270 1.2e-4 x10)")
    p.add_argument("--noise_acc", type=float, default=1.6e-2, help="acc noise density [m/s^2/sqrt(Hz)] (BMI270 1.6e-3 x10)")
    p.add_argument("--gyro_walk", type=float, default=1.0e-4, help="gyro random walk [rad/s^2/sqrt(Hz)]")
    p.add_argument("--acc_walk", type=float, default=1.0e-3, help="acc random walk [m/s^3/sqrt(Hz)]")
    p.add_argument("--features", type=int, default=1200, help="ORBextractor.nFeatures (default 1200)")
    p.add_argument("--ini_fast", type=int, default=20, help="ORBextractor.iniThFAST (lower for dark, low-contrast images)")
    p.add_argument("--min_fast", type=int, default=7, help="ORBextractor.minThFAST")
    p.add_argument("--force", action="store_true", help="overwrite an existing output folder")
    return p.parse_args()


def load_jsonl(path):
    frames, gyro, acc = [], [], []
    with open(path) as f:
        for line in f:
            if not line.strip():
                continue
            m = json.loads(line)
            t = m.get("time")
            if t is None:
                continue
            if "sensor" in m:
                kind, v = m["sensor"].get("type"), m["sensor"].get("values")
                if kind == "gyroscope":
                    gyro.append((t, v[:3]))
                elif kind == "accelerometer":
                    acc.append((t, v[:3]))
            elif "frames" in m or "frame" in m:
                frames.append((t, m.get("number")))
    gyro.sort(key=lambda x: x[0])
    acc.sort(key=lambda x: x[0])
    frames.sort(key=lambda x: x[0])
    return frames, gyro, acc


def find_video(rec, eye):
    for ext in ("mp4", "h264", "mkv", "avi"):
        path = os.path.join(rec, f"rectified_{eye}.{ext}")
        if os.path.exists(path):
            return path
    return None


def read_frames_csv(rec, eye):
    path = os.path.join(rec, f"rectified_{eye}_frames.csv")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return [{"index": int(r["index"]), "seq": int(r["sequence"]), "t": float(r["t_device_s"])} for r in csv.DictReader(f)]


def mat_inv(T):
    import numpy as np
    return np.linalg.inv(np.array(T, dtype=float))


def camera_params(cam, width, height):
    """fx, fy, cx, cy for the video size (scaled if calibration.json was made for another resolution)."""
    fx, fy = cam.get("focalLengthX"), cam.get("focalLengthY")
    cx, cy = cam.get("principalPointX"), cam.get("principalPointY")
    cw, ch = cam.get("imageWidth"), cam.get("imageHeight")
    if None in (fx, fy, cx, cy):
        raise SystemExit("calibration.json: camera without focalLengthX/Y or principalPointX/Y")
    sx = width / cw if cw else 1.0
    sy = height / ch if ch else 1.0
    return fx * sx, fy * sy, cx * sx, cy * sy, (sx, sy)


def interp(series, t):
    """Linear interpolation of [(t, [x,y,z])] at t (clamped at the ends)."""
    ts = [s[0] for s in series]
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return series[0][1]
    if i >= len(series):
        return series[-1][1]
    (t0, v0), (t1, v1) = series[i - 1], series[i]
    a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    return [v0[k] + a * (v1[k] - v0[k]) for k in range(3)]


def opencv_matrix(name, T):
    rows = ",\n         ".join(", ".join(f"{v + 0.0:.9f}" for v in row) for row in T)
    return f"{name}: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: f\n  data: [{rows}]\n"


def main():
    args = parse_args()
    import cv2
    import numpy as np

    rec = args.recording.rstrip("/\\")
    out = args.out or os.path.join(rec, "euroc")
    for need in ("data.jsonl", "calibration.json"):
        if not os.path.exists(os.path.join(rec, need)):
            raise SystemExit(f"{rec}: no {need}")
    if os.path.exists(out):
        if not args.force:
            raise SystemExit(f"{out} exists (use --force to overwrite)")
        shutil.rmtree(out)

    frames, gyro, acc = load_jsonl(os.path.join(rec, "data.jsonl"))
    if len(gyro) < 10 or len(acc) < 10:
        raise SystemExit("data.jsonl: too little IMU data")
    with open(os.path.join(rec, "calibration.json")) as f:
        calib = json.load(f)
    cams = calib.get("cameras", [])
    if len(cams) < 2:
        raise SystemExit("calibration.json: needs two cameras (stereo)")
    for i, c in enumerate(cams[:2]):
        dist = [d for d in (c.get("distortionCoefficients") or []) if abs(d) > 1e-9]
        if c.get("model", "pinhole") != "pinhole" or dist:
            print(f"warning: camera {i} is '{c.get('model')}' with distortion {dist}: the video is rectified, so it is"
                  " treated as an ideal pinhole anyway; check that calibration.json describes the rectified cameras")

    videos = {eye: find_video(rec, eye) for eye in ("left", "right")}
    missing = [e for e, v in videos.items() if not v]
    if missing:
        raise SystemExit(f"no rectified_{missing[0]} video in {rec}: record with ir_vio.py --record (or sai-cli --gray)")

    # decode both videos
    imgs = {}
    for eye, path in videos.items():
        cap = cv2.VideoCapture(path)
        lst = []
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            lst.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame)
        cap.release()
        if not lst:
            raise SystemExit(f"{path}: could not decode any frame")
        imgs[eye] = lst
        print(f"{eye}: {len(lst)} frames decoded from {os.path.basename(path)}")
    height, width = imgs["left"][0].shape[:2]

    # time of every video frame
    report = {"recording": os.path.abspath(rec), "jsonl_frames": len(frames), "gyro": len(gyro), "acc": len(acc),
              "decoded": {e: len(v) for e, v in imgs.items()}, "mask": args.mask}
    csvs = {eye: read_frames_csv(rec, eye) for eye in ("left", "right")}
    pairs = []   # (t [s, IMU clock], left image, right image)
    if csvs["left"] and csvs["right"]:
        for eye in ("left", "right"):
            if len(csvs[eye]) != len(imgs[eye]):
                print(f"warning: {eye}: {len(csvs[eye])} CSV rows but {len(imgs[eye])} decoded frames; using the first"
                      f" {min(len(csvs[eye]), len(imgs[eye]))}")
            seqs = [r["seq"] for r in csvs[eye]]
            report[f"gaps_{eye}"] = sum(1 for a, b in zip(seqs, seqs[1:]) if b != a + 1)
        right = sorted((r["t"], imgs["right"][r["index"]]) for r in csvs["right"] if r["index"] < len(imgs["right"]))
        right_t = [t for t, _ in right]
        left = [(r["t"], r["seq"], imgs["left"][r["index"]]) for r in csvs["left"] if r["index"] < len(imgs["left"])]
        # clock: video device time vs data.jsonl frame time (nearest match), median difference removed
        offset = 0.0
        if frames:
            jt = [t for t, _ in frames]
            diffs = []
            for t, _, _ in left:
                i = bisect.bisect_left(jt, t)
                cand = [jt[k] for k in (i - 1, i) if 0 <= k < len(jt)]
                if cand:
                    diffs.append(t - min(cand, key=lambda x: abs(x - t)))
            if diffs:
                offset = statistics.median(diffs)
                spread = statistics.median(abs(d - offset) for d in diffs)
                report["clock_offset_s"], report["clock_spread_s"] = offset, spread
                print(f"clock: video - data.jsonl = {offset * 1000:.2f} ms (median), spread {spread * 1000:.2f} ms")
                if abs(offset) > 0.5:
                    print("  (different clocks: the offset is removed, check that the IMU and the images stay in sync)")
        # left and right frames are paired by device time (the two mono cameras are hardware-synced)
        unpaired = 0
        for t, _, im in left:
            i = bisect.bisect_left(right_t, t)
            best = min((k for k in (i - 1, i) if 0 <= k < len(right_t)), key=lambda k: abs(right_t[k] - t), default=None)
            if best is not None and abs(right_t[best] - t) < 0.005:
                pairs.append((t - offset, im, right[best][1]))
            else:
                unpaired += 1
        report["unpaired_left"] = unpaired
        if unpaired:
            print(f"warning: {unpaired} left frames without a right frame within 5 ms (dropped)")
        report["time_source"] = "rectified_*_frames.csv"
    else:
        n = min(len(imgs["left"]), len(imgs["right"]), len(frames))
        if not (len(imgs["left"]) == len(imgs["right"]) == len(frames)):
            print(f"warning: left {len(imgs['left'])}, right {len(imgs['right'])}, data.jsonl {len(frames)} frames:"
                  f" without rectified_*_frames.csv they are matched in order, using the first {n}")
        pairs = [(frames[i][0], imgs["left"][i], imgs["right"][i]) for i in range(n)]
        report["time_source"] = "data.jsonl order"
    if len(pairs) < 10:
        raise SystemExit("too few stereo pairs")

    # keep only frames inside the IMU span (ORB-SLAM3 needs IMU before and after every frame)
    t_imu0, t_imu1 = max(gyro[0][0], acc[0][0]), min(gyro[-1][0], acc[-1][0])
    pairs = [p for p in pairs if t_imu0 < p[0] < t_imu1]
    pairs.sort(key=lambda p: p[0])
    report["pairs"] = len(pairs)

    # mask
    rect = None
    if args.mask:
        x0, y0, x1, y1 = [float(v) for v in args.mask.split(",")]
        rect = (int(x0 * width), int(y0 * height), int(x1 * width), int(y1 * height))

    # write images
    for d in ("cam0", "cam1", "imu0"):
        os.makedirs(os.path.join(out, "mav0", d, "data" if d != "imu0" else ""), exist_ok=True)
    names = []
    for t, il, ir in pairs:
        ns = int(round(t * 1e9))
        if names and ns <= names[-1]:
            continue
        for cam, im in (("cam0", il), ("cam1", ir)):
            if rect:
                im = im.copy()
                cv2.rectangle(im, rect[:2], rect[2:], 0, -1)
            cv2.imwrite(os.path.join(out, "mav0", cam, "data", f"{ns}.png"), im)
        names.append(ns)
    with open(os.path.join(out, "times.txt"), "w") as f:
        f.write("\n".join(str(n) for n in names) + "\n")
    for cam in ("cam0", "cam1"):
        with open(os.path.join(out, "mav0", cam, "data.csv"), "w") as f:
            f.write("#timestamp [ns],filename\n" + "".join(f"{n},{n}.png\n" for n in names))

    # IMU: gyro times, acc interpolated
    with open(os.path.join(out, "mav0", "imu0", "data.csv"), "w") as f:
        f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for t, g in gyro:
            if t_imu0 <= t <= t_imu1:
                a = interp(acc, t)
                f.write(f"{int(round(t * 1e9))},{g[0]:.9f},{g[1]:.9f},{g[2]:.9f},{a[0]:.9f},{a[1]:.9f},{a[2]:.9f}\n")
    imu_hz = (len(gyro) - 1) / (gyro[-1][0] - gyro[0][0])
    fps = (len(names) - 1) / ((names[-1] - names[0]) / 1e9)

    # settings: camera 0 = left; T_b_c1 = camera -> IMU (body) = inverse of SAI's imuToCamera
    fx, fy, cx, cy, scale = camera_params(cams[0], width, height)
    fx2, fy2, cx2, cy2, _ = camera_params(cams[1], width, height)
    T_ic0 = mat_inv(cams[0]["imuToCamera"])
    T_right_left = np.array(cams[1]["imuToCamera"], dtype=float) @ T_ic0   # point in left camera -> right camera
    T_c1_c2 = np.linalg.inv(T_right_left)                                   # ORB-SLAM3: right camera -> left camera
    baseline = float(np.linalg.norm(T_right_left[:3, 3]))
    rot_deg = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(T_right_left[:3, :3]) - 1) / 2))))
    if rot_deg > 0.5:
        print(f"warning: {rot_deg:.2f} deg between the cameras: for rectified images it should be ~0")
    report.update({"width": width, "height": height, "fx": fx, "fy": fy, "cx": cx, "cy": cy, "intrinsics_scale": scale,
                   "baseline_m": baseline, "fps": fps, "imu_hz": imu_hz})
    if not 0.02 < baseline < 0.3:
        print(f"warning: baseline {baseline:.3f} m looks wrong (OAK-D Pro W: 0.075 m); check calibration.json")
    yaml = f"""%YAML:1.0
# Written by sai_to_euroc.py from {os.path.abspath(rec)}
File.version: "1.0"

# the images are already rectified: two ideal pinholes (no distortion), the stereo transform from calibration.json
Camera.type: "PinHole"
Camera1.fx: {fx:.6f}
Camera1.fy: {fy:.6f}
Camera1.cx: {cx:.6f}
Camera1.cy: {cy:.6f}
Camera2.fx: {fx2:.6f}
Camera2.fy: {fy2:.6f}
Camera2.cx: {cx2:.6f}
Camera2.cy: {cy2:.6f}
Camera.width: {width}
Camera.height: {height}
Camera.fps: {round(fps)}
Camera.RGB: 0

Stereo.ThDepth: 40.0
# right camera -> left camera; baseline {baseline * 100:.2f} cm
{opencv_matrix("Stereo.T_c1_c2", T_c1_c2)}
# camera 0 (left) -> body (IMU) = inverse of imuToCamera in calibration.json
{opencv_matrix("IMU.T_b_c1", T_ic0)}
IMU.NoiseGyro: {args.noise_gyro:g}
IMU.NoiseAcc: {args.noise_acc:g}
IMU.GyroWalk: {args.gyro_walk:g}
IMU.AccWalk: {args.acc_walk:g}
IMU.Frequency: {imu_hz:.1f}

ORBextractor.nFeatures: {args.features}
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: {args.ini_fast}
ORBextractor.minThFAST: {args.min_fast}

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
Viewer.imageViewScale: 1.0
"""
    with open(os.path.join(out, "orbslam3.yaml"), "w") as f:
        f.write(yaml)
    with open(os.path.join(out, "convert.json"), "w") as f:
        json.dump(report, f, indent=1)
    print(f"{len(names)} stereo pairs ({fps:.1f} fps), IMU {imu_hz:.0f} Hz, baseline {baseline * 100:.1f} cm,"
          f" {width}x{height}, fx {fx:.1f}{', mask ' + args.mask if args.mask else ''} -> {out}")


if __name__ == "__main__":
    main()
