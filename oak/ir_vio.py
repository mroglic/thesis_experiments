"""
VIO (Spectacular AI) on an OAK-D Pro (W) with the IR flood light on: like `sai-cli record oak --preview3d`,
but the IR light can be set and changed while running, so tracking can be tried in the dark.

VIO uses the two mono cameras (OV9282), which see 940 nm IR. The flood LED lights the scene evenly; the dot
projector is off by default, because its dots move with the camera and confuse tracking.

    pip install spectacularAI[full] opencv-python
    python ir_vio.py                    # flood 0.6, dots off, 3D view + IR image
    python ir_vio.py --flood 0          # same without IR light, for comparison
    python ir_vio.py --record out/dark  # also record the session (for sai-cli process / diagnose)

In the IR image window:  + / -  flood brighter / darker,  d  dot projector on/off,  q  quit.
In the 3D window: the visualizer's own keys (q quits, mouse moves the view).
Every VIO pose goes to poses.jsonl (time, position, status, flood) for comparing runs.
"""
import argparse
import json
import os
import threading
import time


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--flood", type=float, default=0.6, help="IR flood light intensity 0..1 (default 0.6)")
    p.add_argument("--dots", type=float, default=0.0, help="IR dot projector intensity 0..1 (default 0 = off)")
    p.add_argument("--resolution", default="400p", choices=["400p", "800p"], help="mono camera resolution for VIO")
    p.add_argument("--no_slam", action="store_true", help="VIO only, no SLAM (no loop closures, no point cloud)")
    p.add_argument("--no_gray", action="store_true", help="do not show the IR image window")
    p.add_argument("--record", help="also record the session to this folder")
    p.add_argument("--log", default="poses.jsonl", help="pose log file (default poses.jsonl, empty = none)")
    return p.parse_args()


def main():
    args = parse_args()
    import depthai
    import spectacularAI
    from spectacularAI.cli.visualization.visualizer import Visualizer, VisualizerArgs

    config = spectacularAI.depthai.Configuration()
    config.inputResolution = args.resolution
    config.useSlam = not args.no_slam
    if args.record:
        os.makedirs(args.record, exist_ok=True)
        config.recordingFolder = args.record

    vis_args = VisualizerArgs()
    vis_args.targetFps = 30
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
        light = {"flood": max(0.0, min(1.0, args.flood)), "dots": max(0.0, min(1.0, args.dots))}
        log = open(args.log, "a") if args.log else None
        with depthai.Device(pipeline) as device, vio_pipeline.startSession(device) as vio_session:
            def apply_light():
                device.setIrFloodLightIntensity(light["flood"])
                device.setIrLaserDotProjectorIntensity(light["dots"])
                print(f"IR flood {light['flood']:.2f} · dots {light['dots']:.2f}")
            apply_light()
            gray = None if args.no_gray else device.getOutputQueue(name="gray-left", maxSize=4, blocking=False)
            last_status = None
            while not should_quit.is_set():
                progress = False
                if gray is not None and gray.has():
                    import cv2
                    frame = gray.get().getCvFrame()
                    text = f"flood {light['flood']:.1f}  dots {light['dots']:.1f}  {last_status or ''}"
                    cv2.putText(frame, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1, cv2.LINE_AA)
                    cv2.imshow("IR (left mono, what VIO sees)", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("+"), ord("=")):
                        light["flood"] = min(1.0, round(light["flood"] + 0.1, 2)); apply_light()
                    elif key == ord("-"):
                        light["flood"] = max(0.0, round(light["flood"] - 0.1, 2)); apply_light()
                    elif key == ord("d"):
                        light["dots"] = 0.0 if light["dots"] > 0 else 0.5; apply_light()
                    elif key == ord("q"):
                        should_quit.set()
                        visualizer.shouldQuit = True   # also closes the 3D window
                    progress = True
                if vio_session.hasOutput():
                    out = vio_session.getOutput()
                    visualizer.onVioOutput(out.getCameraPose(0), status=out.status)
                    status = str(out.status).split(".")[-1]
                    if status != last_status:
                        print("tracking:", status)
                        last_status = status
                    if log:
                        p = out.pose.position
                        log.write(json.dumps({"time": out.pose.time, "position": [p.x, p.y, p.z], "status": status,
                                              "flood": light["flood"], "dots": light["dots"]}) + "\n")
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
