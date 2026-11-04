#!/usr/bin/env python3
"""
Step-response metrics from one or more Task 1.2 bags (for the report table).

For every constant non-zero command segment (the 0.5 m/s forward step and the +-1 rad/s rotations)
it reports: target, 10-90 % rise time, overshoot, settling time (+-band), steady-state mean/error,
RMS tracking error during the hold.  Optionally writes a CSV and a matplotlib PNG (backup figure
if you do not use a Foxglove screenshot).

Usage:
  python3 analyze_step.py ~/bags/task1_2/ff_*  ~/bags/task1_2/tuned_*  --labels FF "FF+PD" \
          --csv metrics.csv --png step_response.png
  --source odom  (default, /odom twist, ~22 Hz)   |  --source meas  (/step_test/meas from /motor_vel, ~90 Hz)
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
from bag_tools import iter_messages, open_reader


def load_series(bag, source):
    """Return dict with cmd (t, vx, wz) and meas (t, vx, wz), t in seconds from bag start."""
    _, types = open_reader(bag)
    cmd_topic = "/step_test/cmd" if "/step_test/cmd" in types else "/cmd_vel"
    meas_topic = {"odom": "/odom", "meas": "/step_test/meas"}[source]
    if meas_topic not in types:
        sys.exit(f"{bag}: topic {meas_topic} not in bag (available: {sorted(types)})")
    cmd, meas = [], []
    for topic, t_ns, msg in iter_messages(bag, [cmd_topic, meas_topic]):
        if topic == cmd_topic:
            tw = msg.twist if cmd_topic == "/step_test/cmd" else msg
            cmd.append((t_ns, tw.linear.x, tw.angular.z))
        else:
            tw = msg.twist.twist if meas_topic == "/odom" else msg.twist
            meas.append((t_ns, tw.linear.x, tw.angular.z))
    if not cmd or not meas:
        sys.exit(f"{bag}: missing data (cmd={len(cmd)}, meas={len(meas)})")
    t0 = min(cmd[0][0], meas[0][0])
    cmd = np.array(cmd, dtype=float); meas = np.array(meas, dtype=float)
    cmd[:, 0] = (cmd[:, 0] - t0) / 1e9
    meas[:, 0] = (meas[:, 0] - t0) / 1e9
    return {"cmd": cmd, "meas": meas}


def find_segments(t, u, min_len=1.0, thr=0.02):
    """Constant non-zero stretches of the command u(t): list of (t_start, t_end, value)."""
    segs, start, val = [], None, 0.0
    for i in range(len(t)):
        if abs(u[i]) > thr:
            if start is None or abs(u[i] - val) > thr:
                if start is not None and t[i - 1] - start >= min_len:
                    segs.append((start, t[i - 1], val))
                start, val = t[i], u[i]
        elif start is not None:
            if t[i - 1] - start >= min_len:
                segs.append((start, t[i], val))   # end = first zero command (receive time)
            start = None
    if start is not None and t[-1] - start >= min_len:
        segs.append((start, t[-1], val))
    return segs


def step_metrics(tm, ym, t_on, t_off, target, band=0.05, ss_window=1.5):
    m = (tm >= t_on) & (tm <= t_off)
    t, y = tm[m] - t_on, ym[m]
    if len(t) < 5:
        return None
    s = np.sign(target); ya = y * s; tg = abs(target)
    def first_time(cond):
        idx = np.nonzero(cond)[0]
        return float(t[idx[0]]) if len(idx) else math.nan
    t10 = first_time(ya >= 0.1 * tg)
    t90 = first_time(ya >= 0.9 * tg)
    peak = float(ya.max())
    overshoot = max(0.0, (peak - tg) / tg * 100.0)
    inside = np.abs(ya - tg) <= band * tg
    settle = math.nan
    for i in range(len(t)):
        if inside[i:].all():
            settle = float(t[i]); break
    ss = ya[t >= (t[-1] - ss_window)]
    return {
        "target": target,
        "rise_10_90_s": t90 - t10 if not (math.isnan(t10) or math.isnan(t90)) else math.nan,
        "t_90_s": t90,
        "overshoot_pct": overshoot,
        "peak": peak * s,
        f"settle_{int(band*100)}pct_s": settle,
        "ss_mean": float(ss.mean()) * s,
        "ss_std": float(ss.std()),
        "ss_err": float(ss.mean()) * s - target,
        "ss_err_pct": (float(ss.mean()) - tg) / tg * 100.0,
        "rms_err_hold": float(np.sqrt(np.mean((ya - tg) ** 2))),
        "n_samples": int(len(t)),
    }


def analyze(bag, source):
    d = load_series(bag, source)
    rows = []
    for axis, col, name in ((1, 1, "vx [m/s]"), (2, 2, "wz [rad/s]")):
        for t_on, t_off, val in find_segments(d["cmd"][:, 0], d["cmd"][:, col]):
            met = step_metrics(d["meas"][:, 0], d["meas"][:, col], t_on, t_off, val)
            if met:
                met.update({"signal": name, "t_on": t_on, "t_off": t_off})
                rows.append(met)
    return d, rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--labels", nargs="+")
    ap.add_argument("--source", choices=["odom", "meas"], default="odom")
    ap.add_argument("--band", type=float, default=0.05, help="settling band as fraction of target")
    ap.add_argument("--csv")
    ap.add_argument("--png", help="write an overlay figure (vx and wz vs time, runs aligned at onset)")
    args = ap.parse_args()
    labels = args.labels or [os.path.basename(os.path.normpath(b)) for b in args.bags]
    if len(labels) != len(args.bags):
        sys.exit("--labels count must match bags")

    all_rows, series = [], []
    for bag, label in zip(args.bags, labels):
        d, rows = analyze(bag, args.source)
        for r in rows:
            r["run"] = label
        all_rows += rows
        series.append((label, d))

    cols = ["run", "signal", "target", "rise_10_90_s", "t_90_s", "overshoot_pct", "peak",
            f"settle_{int(args.band*100)}pct_s", "ss_mean", "ss_std", "ss_err", "ss_err_pct", "rms_err_hold"]
    print("| " + " | ".join(cols) + " |")
    print("|" + "---|" * len(cols))
    for r in all_rows:
        print("| " + " | ".join(f"{r[c]:.3f}" if isinstance(r[c], float) else str(r[c]) for c in cols) + " |")

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols + ["t_on", "t_off", "n_samples"])
            w.writeheader(); w.writerows(all_rows)
        print(f"\ncsv -> {args.csv}")

    if args.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
        for label, d in series:
            seg = find_segments(d["cmd"][:, 0], d["cmd"][:, 1])
            t_on = seg[0][0] if seg else 0.0
            for ax, col, nm in ((axes[0], 1, "vx [m/s]"), (axes[1], 2, "wz [rad/s]")):
                ax.plot(d["meas"][:, 0] - t_on, d["meas"][:, col], label=f"{label} ({args.source})", lw=1.2)
                ax.set_ylabel(nm); ax.grid(True, alpha=0.3)
        l0, d0 = series[0]
        seg = find_segments(d0["cmd"][:, 0], d0["cmd"][:, 1]); t_on = seg[0][0] if seg else 0.0
        axes[0].step(d0["cmd"][:, 0] - t_on, d0["cmd"][:, 1], "k--", where="post", lw=1, label="command")
        axes[1].step(d0["cmd"][:, 0] - t_on, d0["cmd"][:, 2], "k--", where="post", lw=1, label="command")
        axes[0].legend(); axes[1].set_xlabel("time since step onset [s]")
        axes[0].set_title("Task 1.2 step response")
        fig.tight_layout(); fig.savefig(args.png, dpi=150)
        print(f"png -> {args.png}")


if __name__ == "__main__":
    main()
