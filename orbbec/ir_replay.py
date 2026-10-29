"""
Plays a Spectacular AI recording (from ir_record.py or sai-record-orbbec) through the VIO engine with the same
3D view as sai-record-orbbec, so IR-only tracking can be watched as if live. Also works without a window
(--no_vis) and prints a summary: how long it tracked, path length, how far it ended from the start.

While tracking is lost the engine gives no poses and the stock visualizer stops drawing (the window freezes);
here it keeps drawing at the last good pose, so the view can still be zoomed and turned, and stays open at the
end. Lost periods are printed in the console.

    python ir_replay.py recordings\\ir_2026-10-29_13-00-00
    python ir_replay.py recordings\\ir_2026-10-29_13-00-00 --speed -1 --no_vis      # as fast as possible, summary only
    python ir_replay.py recordings\\2026-10-29_12-32-02                             # colour recording, for comparison

Every pose goes to poses.jsonl in the recording folder (time, position, status), for comparing runs.
"""
import argparse
import json
import os
import threading
import time


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("folder", help="recording folder")
    p.add_argument("--speed", type=float, default=1.0, help="playback speed, 1 = real time, -1 = as fast as possible")
    p.add_argument("--no_vis", action="store_true", help="no 3D window, summary only")
    p.add_argument("--no_slam", action="store_true", help="VIO only (no loop closures, no point cloud)")
    p.add_argument("--params", help="override parameter sets, e.g. wrapper-base,icp")
    return p.parse_args()


def main():
    args = parse_args()
    import spectacularAI

    configuration = {}
    if args.no_slam:
        configuration["useSlam"] = "false"
    if args.params:
        configuration["parameterSets"] = "[" + args.params + "]"

    visualizer = None
    if not args.no_vis:
        from spectacularAI.cli.visualization.visualizer import Visualizer, VisualizerArgs
        vis_args = VisualizerArgs()
        vis_args.targetFps = 30
        vis_args.keepOpenAfterFinalMap = True
        visualizer = Visualizer(vis_args)

    stats = {"n": 0, "path": 0.0, "first": None, "last": None, "status": None, "t0": None, "t1": None,
             "lost_since": None, "lost": [], "pose": None}
    done = threading.Event()
    log = open(os.path.join(args.folder, "poses.jsonl"), "w")

    def on_output(output, frames):
        status = str(output.status).split(".")[-1]
        p = output.pose.position
        position = (p.x, p.y, p.z)
        stats["n"] += 1
        stats["t0"] = stats["t0"] if stats["t0"] is not None else output.pose.time
        stats["t1"] = output.pose.time
        if status == "TRACKING":
            if stats["lost_since"] is not None:
                stats["lost"].append((stats["lost_since"], output.pose.time))
                stats["lost_since"] = None
            if stats["last"] is not None:
                stats["path"] += sum((a - b) ** 2 for a, b in zip(position, stats["last"])) ** 0.5
            stats["first"] = stats["first"] or position
            stats["last"] = position
        elif stats["lost_since"] is None:
            stats["lost_since"] = output.pose.time
        if status != stats["status"]:
            print(f"t={output.pose.time - stats['t0']:6.1f} s  {status}")
            stats["status"] = status
        log.write(json.dumps({"time": output.pose.time, "position": position, "status": status}) + "\n")
        if visualizer:
            if status == "TRACKING":
                stats["pose"] = output.getCameraPose(0)
            if stats["pose"] is not None:   # while lost: keep the last good pose so the window keeps drawing
                visualizer.onVioOutput(stats["pose"], status=spectacularAI.TrackingStatus.TRACKING)

    replay = spectacularAI.Replay(args.folder, mapperCallback=visualizer.onMappingOutput if visualizer else None,
                                  configuration=configuration)
    replay.setExtendedOutputCallback(on_output)
    replay.setPlaybackSpeed(args.speed)

    if visualizer:
        def play():
            replay.runReplay()
            done.set()
        threading.Thread(target=play, daemon=True).start()
        visualizer.run()   # closes when the window is closed (q)
    else:
        replay.runReplay()
    replay.close()
    log.close()

    if stats["lost_since"] is not None:
        stats["lost"].append((stats["lost_since"], stats["t1"]))
    duration = (stats["t1"] or 0) - (stats["t0"] or 0)
    lost = sum(b - a for a, b in stats["lost"])
    print(f"\n{duration:.1f} s, tracking {100 * (1 - lost / duration) if duration else 0:.0f} % of the time, "
          f"path {stats['path']:.2f} m")
    for a, b in stats["lost"]:
        print(f"  lost {a - stats['t0']:.1f} - {b - stats['t0']:.1f} s ({b - a:.1f} s)")
    if stats["first"] and stats["last"]:
        gap = sum((a - b) ** 2 for a, b in zip(stats["first"], stats["last"])) ** 0.5
        print(f"end - start distance {gap:.2f} m  (if you came back to where you started, this is the drift)")


if __name__ == "__main__":
    main()
