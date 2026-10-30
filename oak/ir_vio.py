"""
VIO (Spectacular AI) on an OAK-D Pro (W) with the IR flood light on: like `sai-cli record oak --preview3d`,
but the IR light can be set and changed while running, so tracking can be tried in the dark.

VIO uses the two mono cameras (OV9282), which see 940 nm IR. The flood LED lights the scene evenly; the dot
projector is off by default, because its dots move with the camera and confuse tracking.

    pip install spectacularAI[full] opencv-python
    python ir_vio.py                    # flood 0.6, dots off, 3D view + IR image
    python ir_vio.py --flood 0          # same without IR light, for comparison
    python ir_vio.py --record out/dark  # also record the session (for sai-cli process / diagnose)
    python ir_vio.py --follow           # 3D view follows the camera (the visualizer's default)
    python ir_vio.py --low_latency      # pose on every IMU sample, like ARML (the cube then runs ahead of the image)
    python ir_vio.py --test dark_flood  # one run of the light test (light / dark / dark_flood), see TESTS below
    python ir_vio.py --mask             # VIO ignores the projector area (ARML's trackerMasks), see MASK below

TESTS: the same route under three lights, VIO only (no SLAM, so loop closures do not hide drift):
    light       room lights on, flood off
    dark        room dark, flood off
    dark_flood  room dark, flood 0.6
Each run is logged to runs/<test>_<n>.jsonl (next free n); compare them with compare_runs.py.

MASK: ARML (Vio.cs) passes Spectacular AI the internal parameter trackerMasks = 0.314,0.347,0.700,0.748, a
rectangle in normalised image coordinates where the tracker looks for no features, because the projector
lights it. It is read only when the session starts, so it is off unless you start with --mask; then every
log line has "mask" and a test run is logged as runs/<test>_mask_<n>.jsonl. The m key only shows or hides
the rectangle in the IR image (assuming x0,y0,x1,y1; not verified), it does not switch the mask on.

A 20 cm wireframe cube is placed 1 m in front of the camera when tracking starts. It is drawn in the 3D view
and, projected with the VIO pose, in the IR image: if VIO works, the cube stays put on the table while you
move. --no_cube turns it off, --cube_size and --cube_distance change it.

In the IR image window:  + / -  flood brighter / darker,  d  dot projector on/off,
                         r  reset the cube 1 m in front of the camera,  m  show/hide the mask area,  q  quit.
In the 3D window: the visualizer's own keys (q quits, mouse moves the view; the view stays put unless --follow).
The IR window also shows the sensor's auto exposure (exposure time, ISO): in the dark it goes up.
Every VIO pose goes to poses.jsonl (time, position, status, flood, exposure, ISO) for comparing runs; while VIO
sends no poses (tracking lost), a NO_OUTPUT line every 0.1 s.
"""
import argparse
import json
import os
import threading
import time


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--flood", type=float, default=0.6, help="IR flood light intensity 0..1 (default 0.6)")
    p.add_argument("--max_flood", type=float, default=0.8,
                   help="upper limit for the flood (default 0.8: above it the LED driver switches the light off)")
    p.add_argument("--dots", type=float, default=0.0, help="IR dot projector intensity 0..1 (default 0 = off)")
    p.add_argument("--resolution", default="400p", choices=["400p", "800p"], help="mono camera resolution for VIO")
    p.add_argument("--low_latency", action="store_true",
                   help="pose on every IMU sample instead of every frame (as in ARML); poses.jsonl gets hundreds of lines/s")
    p.add_argument("--no_slam", action="store_true", help="VIO only, no SLAM (no loop closures, no point cloud)")
    p.add_argument("--no_gray", action="store_true", help="do not show the IR image window")
    p.add_argument("--ir_pos", default="20,40",
                   help="IR window top-left x,y in screen pixels (default 20,40 = main monitor; e.g. 1940,40 = monitor to the right)")
    p.add_argument("--record", help="also record the session to this folder")
    p.add_argument("--log", default="poses.jsonl", help="pose log file (default poses.jsonl, empty = none)")
    p.add_argument("--follow", action="store_true", help="3D view follows the camera pose (default: fixed view)")
    p.add_argument("--no_cube", action="store_true", help="do not show the AR test cube")
    p.add_argument("--cube_size", type=float, default=0.2, help="cube side in m (default 0.2)")
    p.add_argument("--cube_distance", type=float, default=1.0, help="cube distance in front of the camera in m (default 1)")
    p.add_argument("--mask", nargs="?", const=ARML_MASK, default=None, metavar="X0,Y0,X1,Y1",
                   help=f"VIO ignores this image area (trackerMasks; default ARML's {ARML_MASK}); off unless given")
    p.add_argument("--test", choices=list(TESTS), help="light test run: sets --flood, --no_slam and --log runs/<test>_<n>.jsonl")
    args = p.parse_args()
    if args.test:
        args.flood = TESTS[args.test]
        args.no_slam = True
        name = args.test + ("_mask" if args.mask else "")
        n = 1
        while os.path.exists(os.path.join("runs", f"{name}_{n}.jsonl")):
            n += 1
        args.log = os.path.join("runs", f"{name}_{n}.jsonl")
        print(f"test {args.test}, run {n}: flood {args.flood}, no SLAM, mask {args.mask or 'off'}, log {args.log}")
        print("start on the mark, wait for TRACKING, walk the route, come back to the mark, wait 5 s, press q")
    return args


ARML_MASK = "0.314,0.347,0.700,0.748"   # trackerMasks in ARML's Vio.cs (projector area)
TESTS = {"light": 0.0, "dark": 0.0, "dark_flood": 0.6}   # test name -> flood intensity
IR_WINDOW = "IR (left mono, what VIO sees)"
CUBE_EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def cube_corners(center, size):
    """8 corners of a cube centred on `center`, aligned with the world axes (z up)."""
    import numpy as np
    h = size / 2
    return np.array([[center[0] + x, center[1] + y, center[2] + z] for x in (-h, h) for y in (-h, h) for z in (-h, h)])


def cube_in_front(camera_pose, distance):
    """World point `distance` m along the camera's optical axis."""
    m = camera_pose.getCameraToWorldMatrix()
    return m[0:3, 3] + distance * m[0:3, 2]


def draw_cube_3d(corners):
    from OpenGL.GL import glUseProgram, glLineWidth, glColor3f, glBegin, glEnd, glVertex3f, GL_LINES
    glUseProgram(0)
    glLineWidth(3)
    glColor3f(0.1, 0.4, 1.0)
    glBegin(GL_LINES)
    for a, b in CUBE_EDGES:
        glVertex3f(*corners[a])
        glVertex3f(*corners[b])
    glEnd()


def draw_cube_image(frame, corners, camera_pose):
    """Project the cube into the IR image with the VIO pose and the (rectified) camera intrinsics."""
    import cv2
    import numpy as np
    pts = (camera_pose.getWorldToCameraMatrix() @ np.c_[corners, np.ones(8)].T)[0:3]
    if np.any(pts[2] < 0.05):
        return   # behind or too close to the camera
    uv = (np.array(camera_pose.camera.getIntrinsicMatrix()) @ pts)[0:2] / pts[2]
    for a, b in CUBE_EDGES:
        cv2.line(frame, (int(uv[0, a]), int(uv[1, a])), (int(uv[0, b]), int(uv[1, b])), 255, 2, cv2.LINE_AA)


def main():
    args = parse_args()
    import depthai
    import spectacularAI
    from spectacularAI.cli.visualization.visualizer import Visualizer, VisualizerArgs

    config = spectacularAI.depthai.Configuration()
    config.inputResolution = args.resolution
    config.useSlam = not args.no_slam
    if args.mask:
        config.internalParameters = {"trackerMasks": args.mask}
        print("tracker mask on:", args.mask)
    if args.low_latency:
        try:
            config.lowLatency = True
        except AttributeError:
            print("this spectacularAI version has no lowLatency option, running without it")
    if args.record:
        os.makedirs(args.record, exist_ok=True)
        config.recordingFolder = args.record

    vis_args = VisualizerArgs()
    vis_args.targetFps = 30
    vis_args.cameraFollow = args.follow
    cube = {"corners": None, "pose": None, "move": False}   # shared by the VIO thread and the 3D window
    if not args.no_cube:
        vis_args.customRenderCallback = lambda: cube["corners"] is not None and draw_cube_3d(cube["corners"])
    visualizer = Visualizer(vis_args)
    if config.useSlam:
        config.parameterSets.append("point-cloud")

    pipeline = depthai.Pipeline()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config, lambda out: visualizer.onMappingOutput(out))

    # the rectified left mono image (the IR view VIO works on) as a preview
    if not args.no_gray:
        xout = pipeline.create(depthai.node.XLinkOut)
        xout.setStreamName("gray-left")
        vio_pipeline.stereo.rectifiedLeft.link(xout.input)

    should_quit = threading.Event()

    def main_loop():
        light = {"flood": max(0.0, min(args.max_flood, args.flood)), "dots": max(0.0, min(1.0, args.dots))}
        if args.log and os.path.dirname(args.log):
            os.makedirs(os.path.dirname(args.log), exist_ok=True)
        log = open(args.log, "a") if args.log else None
        with depthai.Device(pipeline) as device, vio_pipeline.startSession(device) as vio_session:
            def apply_light():
                device.setIrFloodLightIntensity(light["flood"])
                device.setIrLaserDotProjectorIntensity(light["dots"])
                print(f"IR flood {light['flood']:.2f} · dots {light['dots']:.2f}")
            apply_light()
            print("USB:", str(device.getUsbSpeed()).split(".")[-1], "(SUPER or better is USB 3; HIGH = USB 2, too slow)")
            gray = None if args.no_gray else device.getOutputQueue(name="gray-left", maxSize=4, blocking=False)
            last_status = None
            sensor = {"exposure_ms": None, "iso": None}
            show_mask = False
            ir_placed = False
            # When tracking is lost, VIO sends no poses at all. So while none come, log a NO_OUTPUT line every
            # 0.1 s; otherwise the log (and compare_runs.py) only sees the time spent tracking.
            last_output = last_gap_line = time.time()
            while not should_quit.is_set():
                progress = False
                now = time.time()
                if now - last_output > 0.3 and now - last_gap_line > 0.1:
                    if last_status != "NO_OUTPUT":
                        print("tracking: NO_OUTPUT (no poses from VIO)")
                        last_status = "NO_OUTPUT"
                    if log:
                        log.write(json.dumps({"wall": now, "status": "NO_OUTPUT", "flood": light["flood"],
                                              "dots": light["dots"], "mask": args.mask, **sensor}) + "\n")
                    last_gap_line = now
                if gray is not None and gray.has():
                    import cv2
                    msg = gray.get()
                    frame = msg.getCvFrame()
                    try:   # the sensor's auto exposure: long exposure / high ISO = little light, motion blur, noise
                        sensor["exposure_ms"] = round(msg.getExposureTime().total_seconds() * 1000, 2)
                        sensor["iso"] = msg.getSensitivity()
                    except (AttributeError, RuntimeError):
                        pass
                    if cube["corners"] is not None and cube["pose"] is not None and last_status == "TRACKING":
                        draw_cube_image(frame, cube["corners"], cube["pose"])
                    text = f"flood {light['flood']:.1f}  dots {light['dots']:.1f}  {last_status or ''}"
                    cv2.putText(frame, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1, cv2.LINE_AA)
                    if sensor["exposure_ms"]:
                        text = f"exposure {sensor['exposure_ms']:.1f} ms  ISO {sensor['iso']}"
                        cv2.putText(frame, text, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1, cv2.LINE_AA)
                    if show_mask or args.mask:
                        x0, y0, x1, y1 = [float(v) for v in (args.mask or ARML_MASK).split(",")]
                        h, w = frame.shape[:2]
                        cv2.rectangle(frame, (int(x0 * w), int(y0 * h)), (int(x1 * w), int(y1 * h)), 255, 1)
                        label = "mask ACTIVE" if args.mask else "mask area (not active, start with --mask)"
                        cv2.putText(frame, label, (int(x0 * w) + 4, int(y0 * h) + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1, cv2.LINE_AA)
                    cv2.imshow(IR_WINDOW, frame)
                    if not ir_placed:
                        cv2.moveWindow(IR_WINDOW, *[int(v) for v in args.ir_pos.split(",")])
                        ir_placed = True
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("+"), ord("=")):
                        light["flood"] = min(args.max_flood, round(light["flood"] + 0.1, 2)); apply_light()
                    elif key == ord("-"):
                        light["flood"] = max(0.0, round(light["flood"] - 0.1, 2)); apply_light()
                    elif key == ord("d"):
                        light["dots"] = 0.0 if light["dots"] > 0 else 0.5; apply_light()
                    elif key == ord("m"):
                        show_mask = not show_mask
                    elif key in (ord("r"), ord("b")):
                        cube["move"] = True
                    elif key == ord("q"):
                        should_quit.set()
                        visualizer.shouldQuit = True   # also closes the 3D window
                    progress = True
                if vio_session.hasOutput():
                    out = vio_session.getOutput()
                    if last_status == "NO_OUTPUT":
                        print(f"VIO poses resumed after {time.time() - last_output:.1f} s")
                    last_output = time.time()
                    camera_pose = out.getCameraPose(0)
                    visualizer.onVioOutput(camera_pose, status=out.status)
                    status = str(out.status).split(".")[-1]
                    if not args.no_cube and status == "TRACKING":
                        cube["pose"] = camera_pose
                        if cube["corners"] is None or cube["move"]:
                            cube["corners"] = cube_corners(cube_in_front(camera_pose, args.cube_distance), args.cube_size)
                            cube["move"] = False
                    if status != last_status:
                        print("tracking:", status)
                        last_status = status
                    if log:
                        p = out.pose.position
                        log.write(json.dumps({"wall": last_output, "time": out.pose.time, "position": [p.x, p.y, p.z], "status": status,
                                              "flood": light["flood"], "dots": light["dots"], "mask": args.mask, **sensor}) + "\n")
                    progress = True
                if not progress:
                    time.sleep(0.005)
            device.setIrFloodLightIntensity(0)
            device.setIrLaserDotProjectorIntensity(0)
        if log:
            log.close()
        if not args.no_gray:
            import cv2
            cv2.destroyAllWindows()

    reader = threading.Thread(target=main_loop)
    reader.start()
    visualizer.run()   # closes when the 3D window is closed
    should_quit.set()
    reader.join()


if __name__ == "__main__":
    main()
