# Simulation Benchmark (drop laptop code here)

This directory hosts the **Gazebo simulation benchmark** that compares RKO-LIO,
LIO-SAM, and FAST-LIO via identical-bag replay across three AWS RoboMaker indoor
environments (Small House, Warehouse, Bookstore).

See the full method and commands in
[`../docs/SIMULATION_GUIDE.md`](../docs/SIMULATION_GUIDE.md).

## What goes here (from the laptop)
- `benchmark/` — replay + `evo` evaluation scripts.
- `configs/` — per-method simulation parameter files.
- `results/` — **small** artifacts only: `*.tum` trajectories, evo result zips,
  and plots. **No rosbags, no large `.pcd` maps.**

## What must NOT go here
- Raw simulation rosbags (`*.db3`, `*.bag`, `*.mcap`)
- Large point-cloud maps (`*.pcd`, `*.ply`)
- `build/` / `install/` / `log/`

> These are excluded by the repository `.gitignore`. Keep heavy data on the drive.

## Transfer from the laptop (example)
```bash
# Run on the laptop; pushes only small code/result files to the Jetson clone:
rsync -aAX --info=progress2 \
  --exclude 'build/' --exclude '*.bag' --exclude '*.db3' \
  --exclude '*.mcap' --exclude '*.pcd' --exclude '*.ply' \
  ~/<your_sim_ws>/  rai@<robot_ip>:/mnt/ssd/publish/rko-lio-indoor-3d-mapping/simulation/
```
Then on the Jetson: `git add simulation/ && git commit -m "add simulation benchmark"`.
