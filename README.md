# Motion Viewer — NPZ / PKL Viewer / Player for URDF & MJCF Humanoid Robots — GIAR fork

A free, browser-based viewer and player for humanoid robot motion. Load a robot via **URDF or MJCF (.xml)** with STL meshes and play back **`.npz` or `.pkl`** motion trajectories — everything runs client-side, no server, no upload, no install.

**[🚀 Live Demo](https://giar-mv.9zteam.pp.ua/)**

> Search keywords: *npz viewer · npz player · pkl viewer · urdf viewer · mjcf viewer · urdf motion player · humanoid motion viewer · Unitree G1 viewer · Isaac Lab npz · GMR pkl · AMASS viewer · mocap viewer*

## Features

- **Any robot** — drag-and-drop URDF *or* MJCF (`.xml`) + STL meshes (Unitree G1, H1, custom humanoids, GMR/MuJoCo models)
- **NPZ playback** — body-pose trajectories (`body_pos_w`, `body_quat_w`) from Isaac Lab / BeyondMimic
- **PKL playback** — GMR/MuJoCo-style files with `root_pos`, `root_rot`, `dof_pos`; built-in forward kinematics handles the DoF → body-pose conversion
- **Cross-format joint mapping** — PKL slots are mapped to URDF joints by name, so files saved against an MJCF whose joint set differs from the URDF (extra/zero-padded slots) still play correctly
- **Zero install** — single HTML file, runs entirely in the browser via Three.js
- **Playback controls** — play/pause, timeline scrubbing, speed (0.25x–2x), frame stepping
- **Camera follow** — auto-track the robot root body
- **Trajectory trail** — toggle a ground-plane trace of the root path
- **Multi-motion** — load many motion files at once, switch with `[` / `]`
- **Type filter** — sidebar tabs to show All / NPZ / PKL
- **Presets** — one-click load of the G1 robot + its 60 demo motions, or the Go2 robot + its 7 demo motions, from the right-hand panel
- **Data annotation** — review motions one by one, mark each keep/discard (`Y`/`N`), and export a CSV of all labels

## Quick Start

1. Open the [live demo](https://giar-mv.9zteam.pp.ua/) or `index.html` locally
2. Drag a **robot folder** (containing a `.urdf` or `.xml` + `.stl` meshes) into the window, **or** click **G1 moves** / **Go2 moves** in the right-hand **Presets** panel to load a robot with all its demo motions at once
3. Drag one or more **`.npz` or `.pkl`** motion files into the window
4. Use the playback bar to control visualization

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Space` | Play / Pause |
| `←` `→` | Step frame backward / forward |
| `[` `]` | Previous / next motion |
| `Y` / `N` | Label current motion keep / discard (auto-advances to next) |

### Demo Assets

A ready-to-use example is included in `demo/`:

| Asset | Description |
|-------|-------------|
| `demo/unitree_g1/` | Unitree G1 humanoid — URDF + 35 STL meshes |
| `demo/dance1_subject2.npz` | Dance motion stub (133 bytes; the full 131s dance is not included) |
| `demo/g1_moves/` | **60 G1 motions** (`.npz`, Karate / Dance / Move / Short / V) — loaded in bulk by the **G1 moves** preset |
| `demo/go2_moves/` | **7 Go2 motions** (`.pkl`, `go2_pace/run/trot/walk0..3`) — loaded in bulk by the **Go2 moves** preset |

Clone the repo, then drag `demo/unitree_g1/` and `demo/dance1_subject2.npz` into the viewer, or click **G1 moves** / **Go2 moves** in the right-hand **Presets** panel to load a robot with its full set of demo motions.

## NPZ Data Format

Body-pose trajectories in the NumPy `.npz` convention from [whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking) / Isaac Lab.

**Required keys:**

| Key | Shape | Description |
|-----|-------|-------------|
| `fps` | `(1,)` | Playback frame rate, e.g. `[50]` |
| `body_pos_w` | `(N, num_bodies, 3)` | World-frame body positions |
| `body_quat_w` | `(N, num_bodies, 4)` | World-frame body orientations (wxyz) |

**Optional keys:** `joint_pos`, `joint_vel`, `body_lin_vel_w`, `body_ang_vel_w`. The G1 preset motions in `demo/g1_moves/` include `joint_pos` (29 DoF), which the viewer maps onto the G1 URDF to compute body poses via FK.

`N` = frame count. `num_bodies` must match the robot description's BFS body count (fixed joints collapsed into parent). Both uncompressed and `deflate-raw`-compressed NPZ files are supported.

## PKL Data Format

GMR / MuJoCo-style Python pickles (protocol 4). The parser is a small subset that handles dict-of-ndarray payloads.

**Required keys:**

| Key | Shape | Description |
|-----|-------|-------------|
| `fps` | scalar | Playback frame rate |
| `root_pos` | `(N, 3)` | Root world position |
| `root_rot` | `(N, 4)` | Root world rotation (**xyzw**, MuJoCo convention) |
| `dof_pos` | `(N, num_dof)` | Per-joint angles in MJCF body-tree DFS order |

Body poses are computed in the browser via forward kinematics walking the URDF/MJCF chain. Both C-contiguous and Fortran-order numpy arrays are handled. PKL slots are matched to URDF joints by name when the save format includes extra (zero-padded) joints not present in the loaded robot — see `PKL_JOINT_PRESETS` in `index.html` for the supported presets.

## Robot Description + Mesh Requirements

- **URDF** with `<visual>` meshes pointing to `.stl` files, **or** **MJCF** (`<mujoco>` root) with `<asset><mesh file="..."/>` entries
- Meshes are resolved by **filename only** (e.g. `package://foo/bar/pelvis.STL` matches `pelvis.stl`)
- Binary STL only
- Description file and meshes can be in subdirectories — drag the top-level folder

## Coordinate Convention

- Motion data uses **Z-up** (Isaac Sim / MuJoCo convention)
- The viewer converts to Y-up for rendering automatically
- NPZ quaternions are **wxyz**, PKL `root_rot` is **xyzw**, both handled internally

## Deployment

Single HTML file, no build step needed.

**GitHub Pages:** Fork this repo → Settings → Pages → deploy from `main` branch.

**Local:** Open `index.html` directly in a browser, or serve with any static server:

```bash
python -m http.server 8000
```

## License

[MIT](LICENSE)
