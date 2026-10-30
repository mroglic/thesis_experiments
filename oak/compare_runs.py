r"""
Compares ir_vio.py runs from their pose logs (one .jsonl per run).

    python compare_runs.py                    # all runs in runs\ (light_1, dark_1, dark_flood_1, ...)
    python compare_runs.py runs\light_1.jsonl runs\dark_flood_1.jsonl

Time runs from the first TRACKING pose to the end; NO_OUTPUT lines (no poses from VIO) count as lost.
For each run: duration, share of time spent tracking, number of lost periods, path length (tracking only),
end–start distance (drift, if the route starts and ends at the same spot) and the median exposure / ISO.
With several runs of a test (light_1, light_2, ...) it also prints the mean per test; runs with the tracker
mask (dark_flood_mask_1, ...) are a separate test.
"""
import glob
import json
import math
import os
import re
import statistics
import sys


def stats(path):
    with open(path) as f:
        rows = [json.loads(line) for line in f if line.strip()]
    for r in rows:   # wall clock (also on NO_OUTPUT lines); old logs only have the pose time
        r["t"] = r.get("wall", r.get("time"))
    # the run starts at the first TRACKING pose (before that VIO is still initialising on the mark)
    start = next((i for i, r in enumerate(rows) if r["status"] == "TRACKING"), None)
    if start is None:
        return None
    rows = rows[start:]
    duration = rows[-1]["t"] - rows[0]["t"]
    tracking_time, lost, path_len = 0.0, 0, 0.0
    first = last = None
    for prev, row in zip(rows, rows[1:]):
        ok = prev["status"] == "TRACKING" and row["status"] == "TRACKING"
        if ok:
            tracking_time += row["t"] - prev["t"]
            path_len += math.dist(prev["position"], row["position"])
        if prev["status"] == "TRACKING" and row["status"] != "TRACKING":
            lost += 1
    tracked = [r for r in rows if r["status"] == "TRACKING"]
    if tracked:
        first, last = tracked[0]["position"], tracked[-1]["position"]
    exposure = [r["exposure_ms"] for r in rows if r.get("exposure_ms")]
    iso = [r["iso"] for r in rows if r.get("iso")]
    return {
        "run": os.path.splitext(os.path.basename(path))[0],
        "s": duration,
        "tracking %": 100 * tracking_time / duration if duration > 0 else None,
        "lost": lost,
        "path m": path_len,
        "end-start m": math.dist(first, last) if first else None,
        "exposure ms": statistics.median(exposure) if exposure else None,
        "ISO": statistics.median(iso) if iso else None,
        "flood": rows[-1].get("flood", 0),
        "mask": "on" if rows[-1].get("mask") else "off",
    }


FORMATS = {"s": "{:.0f}", "tracking %": "{:.0f}", "lost": "{:.3g}", "path m": "{:.2f}", "end-start m": "{:.2f}",
           "exposure ms": "{:.1f}", "ISO": "{:.0f}", "flood": "{:.1f}"}


def mean_per_test(table):
    """light_1, light_2, ... -> one 'light (mean of 2)' row; only for tests with more than one run."""
    groups = {}
    for r in table:
        groups.setdefault(re.sub(r"_\d+$", "", r["run"]), []).append(r)
    means = []
    for name, runs in groups.items():
        if len(runs) < 2:
            continue
        row = {"run": f"{name} (mean of {len(runs)})"}
        for c in FORMATS:
            values = [r[c] for r in runs if r[c] is not None]
            row[c] = statistics.mean(values) if values else None
        row["mask"] = runs[0]["mask"]
        means.append(row)
    return means


def print_table(rows):
    cells = [{c: (r[c] if c in ("run", "mask") else "-" if r[c] is None else FORMATS[c].format(r[c])) for c in r} for r in rows]
    cols = list(cells[0])
    width = {c: max(len(c), *(len(r[c]) for r in cells)) for c in cols}
    print("  ".join(c.ljust(width[c]) for c in cols))
    for r in cells:
        print("  ".join(r[c].ljust(width[c]) for c in cols))


def main():
    paths = sys.argv[1:] or sorted(glob.glob(os.path.join("runs", "*.jsonl")))
    if not paths:
        sys.exit(__doc__.strip())
    table = [s for s in map(stats, paths) if s]
    print_table(table)
    means = mean_per_test(table)
    if means:
        print()
        print_table(means)


if __name__ == "__main__":
    main()
