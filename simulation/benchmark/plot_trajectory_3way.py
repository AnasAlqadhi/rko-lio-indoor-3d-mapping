#!/usr/bin/env python3
"""
3-way IEEE-style trajectory overlay: Ground Truth vs LIO-SAM vs RKO-LIO vs FAST-LIO.
Reads APE RMSE from the evo .txt files so the legend is always current.

Usage:
    python3 plot_trajectory_3way.py <world_tag>
    e.g.  python3 plot_trajectory_3way.py small_house
"""

import sys
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from pathlib import Path

from evo.tools.file_interface import read_bag_trajectory, Rosbag2Reader
from evo.core import sync

BAGS = Path.home() / "simulation_experiment/bags"
OUT = Path.home() / "simulation_experiment/results"

WORLD = sys.argv[1] if len(sys.argv) > 1 else "small_house"

# tag → (merged bag dir, est topic, results-file stem)
SOURCES = {
    "LIO-SAM":  (f"liosam_merged_{WORLD}",  "/lio_sam/mapping/odometry", f"liosam_ape_{WORLD}"),
    "RKO-LIO":  (f"rkolio_merged_{WORLD}",  "/rko_lio/odometry",         f"rkolio_ape_{WORLD}"),
    "FAST-LIO": (f"fastlio_merged_{WORLD}", "/Odometry",                 f"fastlio_ape_{WORLD}"),
}
TOPIC_GT = "/odom"
COLORS = {"LIO-SAM": "#d73027", "RKO-LIO": "#1a9850", "FAST-LIO": "#7b3294"}
STYLES = {"LIO-SAM": "--", "RKO-LIO": "-.", "FAST-LIO": ":"}


def load(bag_path, topic):
    with Rosbag2Reader(str(bag_path)) as reader:
        return read_bag_trajectory(reader, topic)


def ape_rmse(stem):
    f = OUT / f"{stem}.txt"
    if not f.exists():
        return None
    for line in f.read_text().splitlines():
        m = re.search(r"rmse\s+([0-9.]+)", line, re.I)
        if m:
            return float(m.group(1))
    return None


plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 9, "legend.fontsize": 8,
    "xtick.labelsize": 8, "ytick.labelsize": 8, "axes.linewidth": 0.8,
    "lines.linewidth": 1.2, "pdf.fonttype": 42, "ps.fonttype": 42,
})

fig, ax = plt.subplots(figsize=(3.5, 3.2))

gt_plotted = False
for name, (bagdir, topic, stem) in SOURCES.items():
    bagpath = BAGS / bagdir
    if not bagpath.exists():
        print(f"skip {name}: {bagpath} missing")
        continue
    gt = load(bagpath, TOPIC_GT)
    est = load(bagpath, topic)
    gt_s, est_s = sync.associate_trajectories(gt, est)
    est_s.align(gt_s, correct_scale=True)
    if not gt_plotted:
        g = gt_s.positions_xyz[:, :2]
        ax.plot(g[:, 0], g[:, 1], color="#2166ac", lw=1.4, label="Ground Truth", zorder=3)
        ax.plot(g[0, 0], g[0, 1], "o", ms=4, color="#2166ac", zorder=5)
        gt_plotted = True
    e = est_s.positions_xyz[:, :2]
    rmse = ape_rmse(stem)
    lbl = f"{name}  (APE {rmse:.3f} m)" if rmse is not None else name
    ax.plot(e[:, 0], e[:, 1], color=COLORS[name], ls=STYLES[name], lw=1.0, label=lbl, zorder=2)
    ax.plot(e[0, 0], e[0, 1], "o", ms=4, color=COLORS[name], zorder=5)

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title(f"Trajectory Comparison: {WORLD.replace('_', ' ').title()}")
ax.legend(loc="best", framealpha=0.9, edgecolor="0.7")
ax.set_aspect("equal", adjustable="datalim")
ax.grid(True, ls=":", lw=0.4, color="0.75")
fig.tight_layout(pad=0.5)

pdf_out = OUT / f"trajectory_3way_{WORLD}.pdf"
png_out = OUT / f"trajectory_3way_{WORLD}.png"
fig.savefig(pdf_out, dpi=300, bbox_inches="tight")
fig.savefig(png_out, dpi=300, bbox_inches="tight")
print(f"Saved: {pdf_out}")
print(f"Saved: {png_out}")
