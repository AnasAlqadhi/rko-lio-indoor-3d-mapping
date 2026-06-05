# Simulation Guide — Identical-Bag Benchmark (RKO-LIO vs LIO-SAM vs FAST-LIO)

This guide describes the controlled simulation comparison from the paper. The
simulation work was produced on a separate (laptop) workstation; this folder is the
**landing place** for that code and small result artifacts.

> Heavy data policy: **do NOT commit** simulation rosbags, `.pcd` maps, or `build/`
> outputs. Commit only scripts, configs, trajectory files (`.tum`/`.csv`), evo
> results, and final figures.

## Method overview

1. A simulated **TurtleBot3 Waffle Pi** with a Velodyne VLP-16 plugin (16 beams,
   10 Hz) and a 50 Hz IMU plugin is tele-operated through three **AWS RoboMaker**
   indoor environments: **Small House**, **Warehouse**, **Bookstore**.
2. For each environment, the complete sensor stream (`/velodyne_points`,
   `/imu/data`, ground-truth `/odom`) is recorded **once**.
3. The **identical bag** is replayed offline through **RKO-LIO**, **LIO-SAM**, and
   **FAST-LIO** with no modification → byte-identical input for a fair comparison.
4. Estimated trajectories are evaluated against Gazebo wheel-odometry ground truth
   using **evo** (APE + RPE over 1 m, SE(3) Umeyama alignment).

## Environments
- Gazebo Classic 11 (ROS 2 Humble)
- AWS RoboMaker worlds: Small House, Warehouse, Bookstore
  (https://github.com/aws-robotics — Small House / Warehouse / Bookstore worlds)
- Gazebo world files for the indoor scenes are also available in
  `../real_robot/src/turtlebot_rkolio_sim/worlds/`.

## Expected folder layout (fill from the laptop)

```
simulation/
├── README.md
├── benchmark/          # replay + evo evaluation scripts
│   ├── replay_all.sh         # replay one bag through all 3 methods
│   └── evaluate_evo.sh       # run evo_ape / evo_rpe, export tables + plots
├── configs/            # per-method sim parameter files
│   ├── rkolio_sim.yaml
│   ├── liosam_sim.yaml
│   └── fastlio_sim.yaml
└── results/            # SMALL artifacts only (no bags!)
    ├── trajectories/   # *.tum estimated + ground-truth
    └── plots/          # evo figures (also copied to ../media/figures/)
```

## How to evaluate with evo (reference commands)

```bash
pip install evo --upgrade

# Absolute pose error (APE), aligned:
evo_ape tum ground_truth.tum estimate.tum -a --plot --save_results ape.zip

# Relative pose error (RPE) over 1 m:
evo_rpe tum ground_truth.tum estimate.tum -a --delta 1 --delta_unit m --plot

# Overlay trajectories (Fig. 6):
evo_traj tum est_rkolio.tum est_liosam.tum est_fastlio.tum \
  --ref ground_truth.tum -p --plot_mode xy
```

## Reproducing the result tables
The published numbers are in [`../docs/RESULTS.md`](../docs/RESULTS.md) and as CSV in
[`../media/tables/table3_sim_ape_rpe.csv`](../media/tables/table3_sim_ape_rpe.csv).

---

## TODO when copying from the laptop
- [ ] Copy `benchmark/` replay + evo scripts.
- [ ] Copy per-method `configs/` (rkolio/liosam/fastlio sim yaml).
- [ ] Copy `results/trajectories/*.tum` and evo result zips/plots.
- [ ] Export Fig. 5 (maps) and Fig. 6 (trajectories) to `../media/figures/`.
- [ ] Do NOT copy raw sim bags or large `.pcd` maps.
