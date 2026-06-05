# Third-Party / Baseline SLAM Systems

This project uses **RKO-LIO** as the primary odometry method and compares it against
**LIO-SAM** and **FAST-LIO** in the simulation benchmark.

## RKO-LIO — git submodule (upstream, used as-is)

RKO-LIO is used unmodified from its official upstream repository, so it is included
here as a **git submodule**:

| System | Role in this work | Upstream repository | License |
|---|---|---|---|
| **RKO-LIO** | Primary odometry (real robot + sim) | https://github.com/PRBonn/rko_lio | see upstream (MIT) |

```bash
# Clone with submodules:
git clone --recurse-submodules https://github.com/AnasAlqadhi/rko-lio-indoor-3d-mapping.git
# Or, if already cloned:
git submodule update --init --recursive
```

## LIO-SAM and FAST-LIO — the author's own simulation versions

> ⚠️ The LIO-SAM and FAST-LIO used in this paper are the **author's own configured
> versions from the simulation workstation (laptop)** — NOT generic upstream clones.
> They are added directly from that laptop into the `simulation/` part of this
> repository, so the exact configs and modifications that produced the benchmark
> results are preserved. See [`../simulation/README.md`](../simulation/README.md).

This keeps the benchmark reproducible with the *same* code that generated the
paper's numbers, rather than a possibly-different upstream version.

## Custom configurations (this repo, not the submodule)

The parameter files used on **our** Velodyne VLP-16 + Pixhawk platform live here:

- Real robot: `real_robot/src/turtlebot_rkolio_hardware/config/`
- Simulation: `simulation/configs/` (copied from the laptop)
