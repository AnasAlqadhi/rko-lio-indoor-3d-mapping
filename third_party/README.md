# Third-Party SLAM Systems

This project **evaluates and integrates** three external LiDAR–inertial odometry
systems. They are **not redistributed** here — they are included as **git
submodules** pointing to their official upstream repositories, so each retains its
own license and you always get the authentic source.

| System | Role in this work | Upstream repository | License |
|---|---|---|---|
| **RKO-LIO** | Primary odometry (real robot + sim) | https://github.com/PRBonn/rko_lio | see upstream (MIT) |
| **FAST-LIO** | Simulation benchmark baseline | https://github.com/hku-mars/FAST_LIO | GPL-2.0 (upstream) |
| **LIO-SAM** | Simulation benchmark baseline | https://github.com/TixiaoShan/LIO-SAM | BSD-3-Clause (upstream) |

## Getting the submodules

```bash
# Clone with submodules:
git clone --recurse-submodules https://github.com/AnasAlqadhi/rko-lio-indoor-3d-mapping.git

# Or, if already cloned:
git submodule update --init --recursive
```

## Pinned commits

The exact commit each submodule points to is recorded in the superproject (run
`git submodule status` to see the SHAs). Update with:

```bash
git submodule update --remote third_party/rko_lio
```

## Custom configurations

The parameter files used to run each system on **our** Velodyne VLP-16 + Pixhawk
platform (and in simulation) live in this repository, **not** in the submodules:

- Real robot: `real_robot/src/turtlebot_rkolio_hardware/config/`
- Simulation: `real_robot/src/turtlebot_rkolio_sim/config/` and `simulation/configs/`

> Note: FAST-LIO is GPL-2.0. We only reference it as a submodule and provide our
> own config/launch files; we do not copy or relicense its source.
