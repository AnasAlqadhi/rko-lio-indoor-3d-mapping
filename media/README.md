# Media — Figures, Video, and Tables

This folder holds the visual assets for the paper and the repository landing page.
Figures use **fixed filenames** so the README and docs link to them even before the
final images are dropped in. Replace the placeholders with your exported images
using the **exact same names**.

```
media/
├── figures/   # paper figures (PNG/JPG/SVG)
├── video/     # demo videos (file via Git LFS, or external link)
└── tables/    # CSV versions of Table II & III (already filled)
```

---

## Figure checklist & suggestions

Drop each file into `media/figures/` with these names:

| File | Paper | What to show | Capture tips / suggestions |
|---|---|---|---|
| `fig1_platform.png` | Fig. 1 | Annotated TurtleBot Waffle Pi with Velodyne VLP-16 (top), Pixhawk Cube Orange+ (middle), Jetson AGX Orin (housing), OpenCR | Photograph the real robot on a plain background, good lighting; add callout labels + a scale reference. A clean white/grey backdrop reads best in print. |
| `fig2_architecture.png` | Fig. 2 | ROS 2 data-flow: Velodyne (10 Hz) + IMU (50 Hz) → RKO-LIO → odometry + 3D map → robot base + remote operator | Make it a clean vector diagram (draw.io / Inkscape / PowerPoint). Show topic names (`/velodyne_points`, `/mavros/imu/data`, `/rko_lio/odometry`). Export SVG **and** PNG. |
| `fig3_object_validation.png` | Fig. 3 | Center: RKO-LIO point-cloud with 5 objects A–E + green dashed measurement lines; around it: photos of each object | Composite panel. Use RViz screenshot for the cloud; annotate measurement axes in green; place real photos in the border panels. |
| `fig4_realrobot_maps.png` | Fig. 4 | (a) tunnel top view, (b) corridor top/side, (c) basement room top, (d) LIO-SAM "chaotic image" failure | 2×2 grid. Use consistent color map and viewpoint per panel; label (a)–(d). Panel (d) is the dramatic LIO-SAM failure — keep it for contrast. |
| `fig5_sim_maps.png` | Fig. 5 | Grid: rows = Small House / Warehouse / Bookstore; columns = Ground Truth / LIO-SAM / RKO-LIO / FAST-LIO | Keep identical camera angle per row. Highlight the destroyed LIO-SAM bookstore map (bottom row). Add row/column headers. |
| `fig6_sim_trajectories.png` | Fig. 6 | Estimated vs. ground-truth trajectories for the 3 sim environments; legend = APE RMSE (m) | Generate with **evo** (`evo_traj ... --plot` or `evo_ape ... --plot`). One subplot per environment. Keep consistent colors per method across all figures. |

### Optional extra figures (nice for the GitHub page / extended version)
- `fig7_error_bars.png` — bar chart of per-object % error (from `tables/table2_object_lengths.csv`).
- `fig8_ape_comparison.png` — grouped bar chart of APE per method per environment (from `tables/table3_sim_ape_rpe.csv`), log-scale y-axis to show the LIO-SAM bookstore spike.
- `fig9_setup_photos.png` — wiring / mounting close-ups for `docs/HARDWARE.md`.

### Style guidelines (consistency = professional)
- One **fixed color per method** everywhere: e.g. RKO-LIO = blue, FAST-LIO = green, LIO-SAM = red, Ground Truth = black.
- Export at **≥300 DPI** (print) and also a web-sized PNG for the README.
- Prefer **SVG** for diagrams (fig2), **PNG** for point clouds/photos.
- Add scale bars / axes where dimensions matter (fig3, fig4).

---

## Video

Two demo videos are planned (drop into `media/video/`):
1. `realrobot_demo.mp4` — real robot driving + mapping.
2. `rviz_demo.mp4` — RViz live point-cloud / odometry view.

Because video files are heavy, choose one:
- **Git LFS:** `git lfs install && git lfs track "media/video/*.mp4"` (already templated in `.gitattributes`).
- **External link (recommended):** upload to YouTube/Drive and link in `media/video/PLACEHOLDER_demo.md` and the main README. (`.gitignore` excludes `*.mp4` by default.)
