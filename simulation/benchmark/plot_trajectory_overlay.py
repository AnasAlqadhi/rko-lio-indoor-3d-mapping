#!/usr/bin/env python3
"""
IEEE-style combined trajectory overlay: Ground Truth vs LIO-SAM vs RKO-LIO
Output: ~/simulation_experiment/results/trajectory_overlay_ieee.pdf
         ~/simulation_experiment/results/trajectory_overlay_ieee.png  (300 dpi)
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from pathlib import Path

from evo.tools.file_interface import read_bag_trajectory, Rosbag2Reader
from evo.core import sync, metrics
from evo.core.trajectory import PoseTrajectory3D
import evo.core.lie_algebra as lie

# ── paths ──────────────────────────────────────────────────────────────────────
BAGS = Path.home() / "simulation_experiment/bags"
OUT  = Path.home() / "simulation_experiment/results"

LIOSAM_BAG  = BAGS / "liosam_merged_04"
RKOLIO_BAG  = BAGS / "rkolio_merged_04"

TOPIC_GT     = "/odom"
TOPIC_LIOSAM = "/lio_sam/mapping/odometry"
TOPIC_RKOLIO = "/rko_lio/odometry"

# ── load trajectories ──────────────────────────────────────────────────────────
def load(bag_path: Path, topic: str) -> PoseTrajectory3D:
    with Rosbag2Reader(bag_path) as reader:
        return read_bag_trajectory(reader, topic)

print("Loading trajectories...")
gt_from_liosam = load(LIOSAM_BAG, TOPIC_GT)
traj_liosam    = load(LIOSAM_BAG, TOPIC_LIOSAM)
gt_from_rkolio = load(RKOLIO_BAG, TOPIC_GT)
traj_rkolio    = load(RKOLIO_BAG, TOPIC_RKOLIO)

# ── time-sync and SE(3) Umeyama align ─────────────────────────────────────────
from evo.core.trajectory import PosePath3D
from evo.core import lie_algebra
import evo.core.transformations as tr

def align_traj(ref: PoseTrajectory3D, est: PoseTrajectory3D):
    """Sync timestamps then SE(3) Umeyama align est → ref frame."""
    ref_s, est_s = sync.associate_trajectories(ref, est)
    est_s.align(ref_s, correct_scale=False)
    return ref_s, est_s

print("Aligning...")
gt_ls, liosam_aligned = align_traj(gt_from_liosam, traj_liosam)
gt_rk, rkolio_aligned = align_traj(gt_from_rkolio, traj_rkolio)

# XY positions
def xy(traj): return traj.positions_xyz[:, :2]

gt_xy    = xy(gt_ls)          # same ground truth (use liosam's synced version)
ls_xy    = xy(liosam_aligned)
rk_xy    = xy(rkolio_aligned)

# ── IEEE figure style ──────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family":       "serif",
    "font.serif":        ["Times New Roman", "DejaVu Serif"],
    "font.size":         9,
    "axes.labelsize":    9,
    "axes.titlesize":    9,
    "legend.fontsize":   8,
    "xtick.labelsize":   8,
    "ytick.labelsize":   8,
    "axes.linewidth":    0.8,
    "grid.linewidth":    0.4,
    "lines.linewidth":   1.2,
    "pdf.fonttype":      42,   # embeds fonts (required for IEEE submission)
    "ps.fonttype":       42,
})

# Single-column IEEE width: 3.5 in; double-column: 7.16 in
FIG_W, FIG_H = 3.5, 3.2   # inches — single column

fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))

# ── plot ───────────────────────────────────────────────────────────────────────
ax.plot(gt_xy[:, 0],  gt_xy[:, 1],
        color="#2166ac", linewidth=1.4, linestyle="-",
        label="Ground Truth", zorder=3)

ax.plot(ls_xy[:, 0],  ls_xy[:, 1],
        color="#d73027", linewidth=1.0, linestyle="--",
        label="LIO-SAM  (APE RMSE 12.08 m)", zorder=2)

ax.plot(rk_xy[:, 0],  rk_xy[:, 1],
        color="#1a9850", linewidth=1.0, linestyle="-.",
        label="RKO-LIO  (APE RMSE 0.08 m)", zorder=4)

# start markers
for traj_xy, col in [(gt_xy, "#2166ac"), (ls_xy, "#d73027"), (rk_xy, "#1a9850")]:
    ax.plot(traj_xy[0, 0], traj_xy[0, 1],
            marker="o", markersize=4, color=col, zorder=5)

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Trajectory Comparison: Small-House Environment")
ax.legend(loc="best", framealpha=0.9, edgecolor="0.7")
ax.set_aspect("equal", adjustable="datalim")
ax.grid(True, linestyle=":", linewidth=0.4, color="0.75")
ax.xaxis.set_major_locator(ticker.AutoLocator())
ax.yaxis.set_major_locator(ticker.AutoLocator())

fig.tight_layout(pad=0.5)

# ── save ───────────────────────────────────────────────────────────────────────
pdf_out = OUT / "trajectory_overlay_ieee.pdf"
png_out = OUT / "trajectory_overlay_ieee.png"
fig.savefig(pdf_out, dpi=300, bbox_inches="tight")
fig.savefig(png_out, dpi=300, bbox_inches="tight")
print(f"Saved: {pdf_out}")
print(f"Saved: {png_out}")
