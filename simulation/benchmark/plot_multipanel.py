#!/usr/bin/env python3
"""
Combined multi-panel trajectory figure: one column per world
(small_house, warehouse, bookstore), each showing GT vs LIO-SAM vs RKO-LIO vs
FAST-LIO. APE RMSE pulled live from the evo .txt files.

Output: results/trajectory_multipanel.{pdf,png}
"""

import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path

from evo.tools.file_interface import read_bag_trajectory, Rosbag2Reader
from evo.core import sync

BAGS = Path.home() / "simulation_experiment/bags"
OUT = Path.home() / "simulation_experiment/results"

# world tag -> (merged-bag suffix, results-stem)
PANELS = [
    ("small_house", "small_house_06", "small_house"),
    ("warehouse",   "warehouse_06",   "warehouse"),
    ("bookstore",   "bookstore_01",   "bookstore"),
]
METHODS = {
    "LIO-SAM":  ("liosam_merged_{}",  "/lio_sam/mapping/odometry"),
    "RKO-LIO":  ("rkolio_merged_{}",  "/rko_lio/odometry"),
    "FAST-LIO": ("fastlio_merged_{}", "/Odometry"),
}
COLORS = {"LIO-SAM": "#d73027", "RKO-LIO": "#1a9850", "FAST-LIO": "#7b3294"}
STYLES = {"LIO-SAM": "--", "RKO-LIO": "-.", "FAST-LIO": ":"}
TOPIC_GT = "/odom"


def load(bag, topic):
    with Rosbag2Reader(str(bag)) as r:
        return read_bag_trajectory(r, topic)


def ape(stem, meth):
    f = OUT / f"{meth}_ape_{stem}.txt"
    if not f.exists():
        return None
    for line in f.read_text().splitlines():
        m = re.search(r"rmse\s+([0-9.]+)", line, re.I)
        if m:
            return float(m.group(1))
    return None


plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 9, "legend.fontsize": 6.5,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "axes.linewidth": 0.8,
    "lines.linewidth": 1.0, "pdf.fonttype": 42, "ps.fonttype": 42,
})

fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.6))  # IEEE double-column width

for ax, (title, suffix, stem) in zip(axes, PANELS):
    gt_done = False
    for name, (tmpl, topic) in METHODS.items():
        bag = BAGS / tmpl.format(suffix)
        if not bag.exists():
            continue
        gt = load(bag, TOPIC_GT)
        est = load(bag, topic)
        gt_s, est_s = sync.associate_trajectories(gt, est)
        est_s.align(gt_s, correct_scale=True)
        if not gt_done:
            g = gt_s.positions_xyz[:, :2]
            ax.plot(g[:, 0], g[:, 1], color="#2166ac", lw=2.2, alpha=0.5, label="Ground Truth", zorder=1)
            gt_done = True
        e = est_s.positions_xyz[:, :2]
        r = ape(stem, name.lower().replace("-", ""))
        lbl = f"{name} ({r:.3f} m)" if r is not None else name
        ax.plot(e[:, 0], e[:, 1], color=COLORS[name], ls=STYLES[name], lw=0.9, label=lbl, zorder=3)
    ax.set_title(title.replace("_", " ").title())
    ax.set_xlabel("x (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, ls=":", lw=0.4, color="0.8")
    ax.legend(loc="best", framealpha=0.9, edgecolor="0.7")
axes[0].set_ylabel("y (m)")

fig.tight_layout(pad=0.4)
fig.savefig(OUT / "trajectory_multipanel.pdf", dpi=300, bbox_inches="tight")
fig.savefig(OUT / "trajectory_multipanel.png", dpi=300, bbox_inches="tight")
print(f"Saved: {OUT/'trajectory_multipanel.pdf'}")
print(f"Saved: {OUT/'trajectory_multipanel.png'}")
