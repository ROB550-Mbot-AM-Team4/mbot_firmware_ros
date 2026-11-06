#!/usr/bin/env python3
"""Figures for the checkpoint 1 odometry PR, drawn from the measured summary numbers."""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)

# reference palette (light): slots 1-2, chrome
BLUE, ORANGE = "#2a78d6", "#eb6834"
SURFACE, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "text.color": INK,
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.titlecolor": INK,
    "font.size": 10, "axes.titlesize": 12, "axes.titleweight": "bold",
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.axisbelow": True, "legend.frameon": False,
})


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("wrote", path)


def label_bars(ax, bars, fmt, inside=False):
    for b in bars:
        v = b.get_height()
        if inside:
            ax.annotate(fmt(v), (b.get_x() + b.get_width() / 2, v),
                        xytext=(0, -13), textcoords="offset points",
                        ha="center", fontsize=9, color="#ffffff", fontweight="bold")
        else:
            ax.annotate(fmt(v), (b.get_x() + b.get_width() / 2, v),
                        xytext=(0, 3 if v >= 0 else -11), textcoords="offset points",
                        ha="center", fontsize=9, color=INK2)


# ---------- Fig 1: publish rates ----------
topics = ["/imu", "/tf", "/odom"]
nominal = [125.0, 62.5, 31.25]          # 8 ms timer, every 1/2/4 ticks
before = [64.2, 32.5, 16.2]             # /odom + /wheel_odom + /gyro_odom + /tf all reliable
after = [96.2, 48.5, 24.4]              # duplicate odom publishers removed

fig, ax = plt.subplots(figsize=(7.2, 3.8))
x = range(len(topics)); w = 0.36
b1 = ax.bar([i - w / 2 for i in x], before, w * 0.94, color=ORANGE,
            label="Before: 3 identical reliable odom topics + tf")
b2 = ax.bar([i + w / 2 for i in x], after, w * 0.94, color=BLUE,
            label="After: /odom + tf only")
for i, n in enumerate(nominal):
    ax.hlines(n, i - w - 0.04, i + w + 0.04, colors=INK2, linestyles=(0, (4, 2)), linewidth=1.5)
    ax.annotate(f"code rate {n:g} Hz", (i, n), xytext=(0, 4), textcoords="offset points",
                ha="center", fontsize=8.5, color=INK2)
label_bars(ax, b1, lambda v: f"{v:.1f}", inside=True)
label_bars(ax, b2, lambda v: f"{v:.1f}", inside=True)
ax.set_xticks(list(x), topics)
ax.set_ylabel("Measured rate [Hz]")
ax.set_ylim(0, 140)
ax.grid(axis="x", visible=False)
ax.set_title("Firmware publish rates, 8 s window")
ax.legend(loc="upper right", fontsize=8.5)
fig.text(0.0, -0.04, "Each reliable publish waits for an agent ACK (~2.5 ms). Removing two reliable "
         "publishes per odom tick shortened the cycle from 15.5 ms to 10.4 ms.",
         fontsize=8.5, color=INK2)
save(fig, "fig1_publish_rates.png")

# ---------- Fig 2: two-turn spin heading error ----------
truth = 4 * math.pi
names = ["Host gyro integral\n(/imu, stamp dt)", "Firmware gyro heading\n(/odom)", "Firmware wheel odom\n(/wheel_odom)"]
before_h = [12.5443, 11.7270, 11.6611]
after_h = [12.6123, 12.5822, 12.3867]
pct = lambda h: (h - truth) / truth * 100.0
bef = [pct(h) for h in before_h]
aft = [pct(h) for h in after_h]

fig, ax = plt.subplots(figsize=(7.2, 3.8))
x = range(len(names)); w = 0.36
b1 = ax.bar([i - w / 2 for i in x], bef, w * 0.94, color=ORANGE, label="Before: fixed 10 ms dt, loop at ~95 Hz")
b2 = ax.bar([i + w / 2 for i in x], aft, w * 0.94, color=BLUE, label="After: measured dt, fixed 100 Hz timer")
ax.axhline(0, color=AXIS, linewidth=1)
label_bars(ax, b1, lambda v: f"{v:+.2f}%")
label_bars(ax, b2, lambda v: f"{v:+.2f}%")
ax.set_xticks(list(x), names)
ax.set_ylabel("Heading error vs 4π [%]")
ax.set_ylim(-9, 2)
ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:+.0f}%" if v else "0"))
ax.grid(axis="x", visible=False)
ax.set_title("Two in-place turns (teleop): accumulated heading error")
ax.legend(loc="lower left", fontsize=8.5)
fig.text(0.0, -0.06, "Same gyro samples: host integration was right, firmware was ~7% short. "
         "After the fix the remaining wheel-odom error (−1.4%) is pure geometry (R/B).",
         fontsize=8.5, color=INK2)
save(fig, "fig2_spin_heading_error.png")

# ---------- Fig 3: control loop period ----------
st = dict(min=10.386, p10=10.443, median=10.475, p90=11.252, max=11.665)
fig, ax = plt.subplots(figsize=(7.2, 2.2))
ax.hlines(0, st["min"], st["max"], colors=BLUE, linewidth=2)
ax.add_patch(plt.Rectangle((st["p10"], -0.18), st["p90"] - st["p10"], 0.36, color=BLUE, alpha=0.35, linewidth=0))
ax.plot([st["median"]], [0], marker="o", markersize=8, color=BLUE, markeredgecolor=SURFACE, markeredgewidth=2)
ax.axvline(10.0, color=INK2, linestyle=(0, (4, 2)), linewidth=1.5)
ax.annotate("nominal 10.00 ms", (10.0, 0.3), ha="left", xytext=(4, 0), textcoords="offset points", fontsize=8.5, color=INK2)
ax.annotate(f"median {st['median']:.2f} ms (95.5 Hz)", (st["median"], 0), xytext=(0, -28),
            textcoords="offset points", ha="center", fontsize=8.5, color=INK2)
ax.annotate(f"p10–p90 {st['p10']:.2f}–{st['p90']:.2f}, max {st['max']:.2f}", (st["p90"], 0.18),
            xytext=(0, 5), textcoords="offset points", ha="center", fontsize=8.5, color=INK2)
ax.set_xlim(9.8, 11.9); ax.set_ylim(-0.6, 0.6)
ax.set_yticks([]); ax.spines["left"].set_visible(False)
ax.grid(axis="y", visible=False)
ax.set_xlabel("Control-loop step [ms] (from /imu header stamps, 538 single steps)")
ax.set_title("Control loop ran slower than nominal (before fix)")
save(fig, "fig3_loop_period.png")
