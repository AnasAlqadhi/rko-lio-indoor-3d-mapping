#!/usr/bin/env python3
"""
Offline qualitative map renderer.

For a given world, reconstruct the map each algorithm "built" by transforming
every raw /velodyne_points scan by that algorithm's estimated pose (from its
odometry output bag) and accumulating. A drifting trajectory smears the map;
an accurate one yields sharp structure. Renders a top-down density image per
algorithm (+ ground truth) as PNG.

Note on frames: a constant body->lidar extrinsic only rigidly shifts the whole
map, it cannot cause smearing. Smearing comes from trajectory inconsistency over
time. So for a *qualitative* sharp-vs-smeared comparison we apply the world<-body
pose directly to the raw scan; the result faithfully shows map consistency.

Usage:
    python3 render_maps.py <world_suffix>
    e.g. python3 render_maps.py small_house_06
"""

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

BAGS = Path.home() / "simulation_experiment/bags"
OUT = Path.home() / "simulation_experiment/results"
WORLD = sys.argv[1]                       # e.g. small_house_06
WORLD_TAG = WORLD.rsplit("_", 1)[0]       # small_house

# algorithm -> (output bag dir, odom topic)
ALGOS = {
    "GroundTruth": (f"sim_{WORLD}", "/odom"),
    "LIO-SAM":     (f"liosam_output_{WORLD}", "/lio_sam/mapping/odometry"),
    "RKO-LIO":     (f"rkolio_output_{WORLD}", "/rko_lio/odometry"),
    "FAST-LIO":    (None, "/Odometry"),   # bag resolved below (baked preferred)
}

POINT_SUBSAMPLE = 3      # keep 1 in N points per scan
MAX_RANGE = 25.0
Z_SLICE = (0.4, 2.0)     # world-height band (m): keeps walls/furniture, drops floor+ceiling clutter


def quat_to_R(x, y, z, w):
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w),   s*(x*z+y*w)],
        [s*(x*y+z*w),   1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w),   s*(y*z+x*w),   1-s*(x*x+y*y)],
    ])


def read_traj(bag_dir, topic):
    """Return (times[N], T[N,4,4])."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    times, mats = [], []
    while reader.has_next():
        t, data, _ = reader.read_next()
        if t != topic:
            continue
        m = deserialize_message(data, Odometry)
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = [p.x, p.y, p.z]
        times.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        mats.append(T)
    return np.array(times), np.array(mats)


def read_scans(bag_dir):
    """Return list of (time, Nx3 points) for /velodyne_points."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    scans = []
    while reader.has_next():
        t, data, _ = reader.read_next()
        if t != "/velodyne_points":
            continue
        m = deserialize_message(data, PointCloud2)
        pts = pc2.read_points_numpy(m, field_names=["x", "y", "z"])
        pts = pts[::POINT_SUBSAMPLE]
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r > 0.5) & (r < MAX_RANGE)]
        scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, pts))
    return scans


def accumulate(scans, times, mats):
    """Transform each scan by nearest-in-time pose; return accumulated Nx2 (xy)."""
    if len(times) == 0:
        return np.empty((0, 2))
    order = np.argsort(times)
    times, mats = times[order], mats[order]
    out = []
    for st, pts in scans:
        idx = np.searchsorted(times, st)
        idx = min(max(idx, 0), len(times) - 1)
        if abs(times[idx] - st) > 0.3:        # no pose near this scan
            continue
        T = mats[idx]
        w = (T[:3, :3] @ pts.T).T + T[:3, 3]
        out.append(w)
    if not out:
        return np.empty((0, 2))
    w = np.vstack(out)
    # horizontal slice in world height -> clean floor-plan outlines
    zc = w[:, 2]
    z0 = np.median(zc)               # robust floor reference (handles frame z-origin diffs)
    mask = (zc > z0 + Z_SLICE[0]) & (zc < z0 + Z_SLICE[1])
    return w[mask, :2]


def main():
    # resolve FAST-LIO bag: prefer baked (clean) trajectory over live-fixer run
    for cand in (f"fastlio_baked_{WORLD}", f"fastlio_merged_{WORLD}",
                 f"fastlio_output_{WORLD}_baked", f"fastlio_output_{WORLD}"):
        if (BAGS / cand).exists():
            ALGOS["FAST-LIO"] = (cand, "/Odometry")
            print(f"FAST-LIO trajectory from: {cand}")
            break

    print(f"Reading raw scans for {WORLD} ...")
    scans = read_scans(BAGS / f"sim_{WORLD}")
    print(f"  {len(scans)} scans")

    names = list(ALGOS.keys())
    fig, axes = plt.subplots(1, len(names), figsize=(3.0 * len(names), 3.0))
    for ax, name in zip(axes, names):
        bagdir, topic = ALGOS[name]
        if bagdir is None or not (BAGS / bagdir).exists():
            ax.set_title(f"{name}\n(missing)")
            ax.axis("off")
            continue
        times, mats = read_traj(BAGS / bagdir, topic)
        xy = accumulate(scans, times, mats)
        print(f"  {name}: {len(times)} poses -> {len(xy)} map points")
        if len(xy):
            ax.scatter(xy[:, 0], xy[:, 1], s=0.04, c="#1a1a2e", alpha=0.25,
                       edgecolors="none", rasterized=True)
        ax.set_title(name, fontsize=11)
        ax.set_aspect("equal")
        ax.set_facecolor("white")
        ax.set_xticks([]); ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_edgecolor("0.7")
    fig.suptitle(f"Reconstructed map — {WORLD_TAG.replace('_', ' ').title()}", y=1.02)
    fig.tight_layout()
    png = OUT / f"map_grid_{WORLD}.png"
    fig.savefig(png, dpi=200, bbox_inches="tight", facecolor="white")
    print(f"Saved: {png}")


if __name__ == "__main__":
    main()
