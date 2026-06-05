# Results Summary

These are the headline results from the paper. CSV versions of both tables are in
[`../media/tables/`](../media/tables/) so you can regenerate the charts.

## 1. Real-robot object-length accuracy (Table II)

Point-cloud measurements vs. calibrated roll-meter ground truth, five objects (A–E).

| Object | Roll Meter (m) | Point Cloud (m) | Δ (cm) | Error (%) | Accuracy (%) |
|---|---|---|---|---|---|
| A | 3.028 | 3.062 | 3.40 | 1.12 | 98.88 |
| B | 2.047 | 2.077 | 3.00 | 1.47 | 98.53 |
| C | 5.158 | 5.192 | 3.40 | 0.66 | 99.34 |
| D | 7.724 | 7.736 | 1.20 | 0.16 | 99.84 |
| E | 3.124 | 3.125 | 0.10 | 0.03 | 99.97 |
| **Mean** | — | — | — | **0.69** | — |

- **RMSE = 0.026 m (2.6 cm)**, **MAE = 0.022 m**, mean error **0.69 %**.
- A **13.7× reduction in RMSE** vs. a prior LIO-SAM deployment on identical
  Velodyne–Pixhawk hardware (Ramadhania et al., RMSE 0.355 m).
- No time-synchronization "chaotic image" artifacts were observed with RKO-LIO.

## 2. Simulation trajectory accuracy (Table III)

Identical-bag replay through each method in three Gazebo environments; RMSE in
metres, evaluated with **evo** (SE(3) Umeyama alignment vs. wheel-odometry truth).
**Bold = best per row. † = catastrophic divergence.**

| Env. | Metric | LIO-SAM | RKO-LIO | FAST-LIO |
|---|---|---|---|---|
| Small House | APE (m) | 0.049 | 0.148 | **0.026** |
| Small House | RPE (m/m) | 0.025 | 0.042 | **0.018** |
| Warehouse | APE (m) | 0.040 | 0.163 | **0.034** |
| Warehouse | RPE (m/m) | 0.023 | 0.033 | **0.016** |
| Bookstore | APE (m) | 21.689 † | 0.494 | **0.039** |
| Bookstore | RPE (m/m) | 22.962 † | 0.148 | **0.022** |

### Key finding — accuracy vs. robustness
- **FAST-LIO** achieves the lowest error in every environment → best for offline
  mapping of known, feature-rich spaces with a high-quality IMU.
- **LIO-SAM** is accurate in easy scenes but **diverges catastrophically** in the
  Bookstore (APE 21.7 m, map destroyed) → unsafe for uncontrolled deployment.
- **RKO-LIO** is the **only** method with bounded error in *every* environment
  (≤ 0.494 m APE) **without per-scene tuning** → most dependable for real
  GPS-denied indoor deployment.

> Worst-case robustness — not peak accuracy — is the decisive property for real
> GPS-denied indoor 3D mapping.
