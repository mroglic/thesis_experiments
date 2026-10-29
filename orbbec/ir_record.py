"""
Records an Orbbec Femto Mega session for Spectacular AI with the ToF IR image in place of the colour image,
so VIO can be tried on active IR only (works in the dark). The Spectacular AI Orbbec plugin always tracks on
the colour camera; this writes the same recording format as sai-record-orbbec, which ir_replay.py then
plays through spectacularAI.Replay (the only public way to feed own data into the engine).

    pip install pyorbbecsdk2 opencv-python numpy            (+ ffmpeg on PATH)
    python ir_record.py                         # 512x512 @ 30 fps, preview window, q stops
    python ir_record.py --duration 60 --no_preview
    python ir_replay.py recordings\\ir_<time>

Recording folder (same layout as sai-record-orbbec):
    data.jsonl        IMU samples and frame times (device clock, s)
    data.mkv          camera 0: ToF IR, 16-bit scaled to 8-bit (sqrt curve up to --ir_max), H.264
    data2.mkv         camera 1: ToF depth, 16-bit mm, FFV1 lossless
    calibration.json  both cameras = the ToF camera (IR and depth come from the same sensor, so they are
                      aligned by construction); intrinsics from the device, imuToCamera from a
                      sai-record-orbbec calibration (camera 1 there is the same ToF camera)
    vio_config.yaml   useStereo: false, so depth is not treated as a second camera: IR and depth have the same
                      calibration, and as a "stereo pair" with zero baseline tracking breaks within seconds

IMU samples come straight from the accelerometer and gyroscope sensors (1 kHz), the same raw values the plugin
records, so the plugin's imuToCamera applies unchanged.
"""
import argparse
import glob
import heapq
import json
import os
import subprocess
import threading
import time

import numpy as np


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    p.add_argument("--out", help="recording folder (default recordings/ir_<time> next to this script)")
    p.add_argument("--calib", help="calibration.json from a sai-record-orbbec recording (default: newest under recordings/)")
    p.add_argument("--recordings", default=os.path.join(here, "recordings"), help="where recordings live")
    p.add_argument("--resolution", default="512x512", choices=["320x288", "512x512", "640x576", "1024x1024"],
                   help="ToF mode: 512x512 / 1024x1024 wide FOV, 320x288 / 640x576 narrow FOV (default 512x512)")
    p.add_argument("--fps", type=int, default=30, choices=[5, 15, 25, 30])
    p.add_argument("--ir_max", type=float, default=1500, help="IR value that maps to white (default 1500)")
    p.add_argument("--no_depth", action="store_true", help="IR only, no depth stream")
    p.add_argument("--params", default="wrapper-base,orbbec-femto,icp", help="Spectacular AI parameter sets for vio_config.yaml")
    p.add_argument("--duration", type=float, default=0, help="stop after this many seconds (0 = until q / Ctrl+C)")
    p.add_argument("--no_preview", action="store_true", help="no IR preview window")
    return p.parse_args()


def find_calibration(args):
    if args.calib:
        return args.calib
    candidates = []
    for path in glob.glob(os.path.join(args.recordings, "*", "calibration.json")):
        with open(path) as f:
            if len(json.load(f).get("cameras", [])) == 2 and not os.path.basename(os.path.dirname(path)).startswith("ir_"):
                candidates.append(path)
    if not candidates:
        raise SystemExit("No sai-record-orbbec calibration found; record a few seconds with start_vio.bat first, or pass --calib")
    return max(candidates, key=os.path.getmtime)


def ffmpeg_writer(path, width, height, fps, pix_fmt, codec_args):
    return subprocess.Popen(
        ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", pix_fmt, "-s", f"{width}x{height}",
         "-r", str(fps), "-i", "-"] + codec_args + [path],
        stdin=subprocess.PIPE)


class JsonlWriter:
    """Writes events in time order: IMU callbacks arrive on other threads and a little later than frames,
    so events are held for `delay` seconds and written sorted."""

    def __init__(self, path, delay=0.3):
        self.file = open(path, "w")
        self.delay = delay
        self.heap = []
        self.count = 0
        self.lock = threading.Lock()
        self.latest = 0.0
        self.write_now({"sdkVersion": "1.53.2", "version": "1.0.0"})

    def write_now(self, event):
        self.file.write(json.dumps(event, separators=(",", ":")) + "\n")

    def add(self, t, event):
        with self.lock:
            heapq.heappush(self.heap, (t, self.count, event))
            self.count += 1
            self.latest = max(self.latest, t)
            while self.heap and self.heap[0][0] < self.latest - self.delay:
                self.write_now(heapq.heappop(self.heap)[2])

    def close(self):
        with self.lock:
            while self.heap:
                self.write_now(heapq.heappop(self.heap)[2])
            self.file.close()


def main():
    args = parse_args()
    from pyorbbecsdk import (Config, OBFormat, OBFrameAggregateOutputMode, OBSensorType, Pipeline)

    width, height = map(int, args.resolution.split("x"))
    calib_path = find_calibration(args)
    with open(calib_path) as f:
        imu_to_tof = json.load(f)["cameras"][1]["imuToCamera"]
    out = args.out or os.path.join(args.recordings, time.strftime("ir_%Y-%m-%d_%H-%M-%S"))
    os.makedirs(out, exist_ok=True)

    pipe = Pipeline()
    device = pipe.get_device()
    config = Config()
    config.enable_stream(pipe.get_stream_profile_list(OBSensorType.IR_SENSOR).get_video_stream_profile(width, height, OBFormat.Y16, args.fps))
    if not args.no_depth:
        config.enable_stream(pipe.get_stream_profile_list(OBSensorType.DEPTH_SENSOR).get_video_stream_profile(width, height, OBFormat.Y16, args.fps))
    config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
    pipe.enable_frame_sync()

    jsonl = JsonlWriter(os.path.join(out, "data.jsonl"))
    started = threading.Event()   # IMU before the first frame is dropped (the first samples carry stale times)
    last_imu = {"accelerometer": 0.0, "gyroscope": 0.0}

    def on_imu(kind):
        def callback(frame):
            f = frame.as_accel_frame() if kind == "accelerometer" else frame.as_gyro_frame()
            t = f.get_timestamp_us() / 1e6
            if not started.is_set() or t <= last_imu[kind]:
                return
            last_imu[kind] = t
            jsonl.add(t, {"sensor": {"type": kind, "values": [f.get_x(), f.get_y(), f.get_z()]}, "time": t})
        return callback

    def imu_profile(sensor, kind, wanted_range):
        profiles = sensor.get_stream_profile_list()
        profiles = [profiles.get_stream_profile_by_index(i) for i in range(profiles.get_count())]
        typed = [(p, p.as_accel_stream_profile() if kind == "accelerometer" else p.as_gyro_stream_profile()) for p in profiles]
        at_1khz = [(p, t) for p, t in typed if str(t.get_sample_rate()).endswith("1_KHZ")]
        best = [p for p, t in at_1khz if str(t.get_full_scale_range()).endswith(wanted_range)]
        return (best or [p for p, _ in at_1khz])[0]

    imu_sensors = []
    for kind, sensor_type, wanted in (("accelerometer", OBSensorType.ACCEL_SENSOR, "4g"), ("gyroscope", OBSensorType.GYRO_SENSOR, "1000dps")):
        sensor = device.get_sensor(sensor_type)
        sensor.start(imu_profile(sensor, kind, wanted), on_imu(kind))
        imu_sensors.append(sensor)

    pipe.start(config)
    intr, dist = (lambda p: (p.depth_intrinsic, p.depth_distortion))(pipe.get_camera_param())
    tof = {"focalLengthX": intr.fx, "focalLengthY": intr.fy, "principalPointX": intr.cx, "principalPointY": intr.cy,
           "imageWidth": width, "imageHeight": height,
           "distortionCoefficients": [dist.k1, dist.k2, dist.p1, dist.p2, dist.k3, dist.k4, dist.k5, dist.k6],
           "model": "brown-conrady", "imuToCamera": imu_to_tof}
    with open(os.path.join(out, "calibration.json"), "w") as f:
        json.dump({"cameras": [tof] if args.no_depth else [tof, tof]}, f, indent=2)
    with open(os.path.join(out, "vio_config.yaml"), "w") as f:
        f.write(f"useSlam: True\nuseStereo: false\nparameterSets: [{args.params}]\n")

    ir_video = ffmpeg_writer(os.path.join(out, "data.mkv"), width, height, args.fps, "gray",
                             ["-c:v", "libx264", "-preset", "ultrafast", "-crf", "12", "-pix_fmt", "yuv420p"])
    depth_video = None if args.no_depth else ffmpeg_writer(os.path.join(out, "data2.mkv"), width, height, args.fps, "gray16le",
                                                           ["-c:v", "ffv1", "-pix_fmt", "gray16le"])
    lut = (np.sqrt(np.clip(np.arange(65536) / args.ir_max, 0, 1)) * 255).astype(np.uint8)

    print(f"Recording to {out}  (calibration imuToCamera from {calib_path})")
    if not args.no_preview:
        import cv2
    number, t_start, last_t = 0, time.time(), None
    try:
        while not args.duration or time.time() - t_start < args.duration:
            frames = pipe.wait_for_frames(200)
            if frames is None:
                continue
            ir = frames.get_ir_frame()
            depth = None if args.no_depth else frames.get_depth_frame()
            if ir is None or (depth is None and not args.no_depth):
                continue
            t = ir.get_timestamp_us() / 1e6
            if last_t is not None and t <= last_t:
                continue
            last_t = t
            started.set()
            ir8 = lut[np.frombuffer(ir.get_data(), dtype=np.uint16)].reshape(height, width)
            ir_video.stdin.write(ir8.tobytes())
            entries = [{"cameraInd": 0, "height": height, "width": width}]
            if depth_video:
                depth_video.stdin.write(np.frombuffer(depth.get_data(), dtype=np.uint16).tobytes())
                entries.append({"cameraInd": 1, "depthScale": 0.001 * depth.get_depth_scale(), "hasDepth": True,
                                "height": height, "width": width})
            jsonl.add(t, {"frames": entries, "number": number, "time": t})
            number += 1
            if not args.no_preview:
                view = cv2.cvtColor(ir8, cv2.COLOR_GRAY2BGR)
                cv2.putText(view, f"REC {number} frames  {time.time() - t_start:.0f}s  q = stop", (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.imshow("Femto Mega ToF IR (what VIO will see)", view)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        for sensor in imu_sensors:
            sensor.stop()
        jsonl.close()
        for video in (ir_video, depth_video):
            if video:
                video.stdin.close()
                video.wait()
        if not args.no_preview:
            cv2.destroyAllWindows()
    print(f"{number} frames, {time.time() - t_start:.0f} s -> {out}")


if __name__ == "__main__":
    main()
