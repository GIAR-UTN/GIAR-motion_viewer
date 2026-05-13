# Motion Viewer — NPZ Viewer / Player for URDF Humanoid Robots

A free, browser-based **NPZ viewer** and **NPZ player** for visualizing humanoid robot motion. Load any URDF with STL meshes and play back `.npz` motion trajectories — everything runs client-side, no server, no upload, no install.

**[🚀 Live Demo](https://zelin-tao.github.io/motion-viewer/)**

> Search keywords: *npz viewer · npz player · npz file viewer · urdf viewer · urdf motion player · humanoid motion viewer · Unitree G1 viewer · Isaac Lab npz · AMASS viewer · mocap viewer*

## Features

- **Any robot** — drag-and-drop any URDF + STL mesh bundle (Unitree G1, H1, custom humanoids)
- **NPZ motion playback** — visualize body pose trajectories at arbitrary FPS
- **Zero install** — single HTML file, runs entirely in the browser via Three.js
- **Playback controls** — play/pause, timeline scrubbing, speed (0.25x–2x), frame stepping
- **Camera follow** — auto-track the robot root body
- **Trajectory trail** — toggle a ground-plane trace of the root path
- **Multi-motion** — load multiple `.npz` files and switch between them

## Quick Start

1. Open the [live demo](https://zelin-tao.github.io/motion-viewer/) or `index.html` locally
2. Drag a **robot folder** (containing `.urdf` + `.stl` meshes) into the window
3. Drag one or more **`.npz` motion files** into the window
4. Use the playback bar to control visualization

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Space` | Play / Pause |
| `←` `→` | Step frame backward / forward |
| `[` `]` | Previous / next motion |

### Demo Assets

A ready-to-use example is included in `demo/`:

| Asset | Description |
|-------|-------------|
| `demo/unitree_g1/` | Unitree G1 humanoid — URDF + 35 STL meshes |
| `demo/dance1_subject2.npz` | Dance motion (131s, 6574 frames @ 50fps, 30 bodies) |

Clone the repo, then drag `demo/unitree_g1/` and `demo/dance1_subject2.npz` into the viewer.

## NPZ Data Format

Motion files use the NumPy `.npz` format, following the convention from [whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking).

**Required keys:**

| Key | Shape | Description |
|-----|-------|-------------|
| `fps` | `(1,)` | Playback frame rate, e.g. `[50]` |
| `body_pos_w` | `(N, num_bodies, 3)` | World-frame body positions |
| `body_quat_w` | `(N, num_bodies, 4)` | World-frame body orientations (wxyz) |

**Optional keys:**

| Key | Shape | Description |
|-----|-------|-------------|
| `joint_pos` | `(N, num_joints)` | Joint positions |
| `joint_vel` | `(N, num_joints)` | Joint velocities |
| `body_lin_vel_w` | `(N, num_bodies, 3)` | Body linear velocities |
| `body_ang_vel_w` | `(N, num_bodies, 3)` | Body angular velocities |

`N` = number of frames. `num_bodies` must match the URDF body count (BFS traversal order, fixed joints collapsed into parent).

## URDF + Mesh Requirements

- Standard URDF with `<visual>` meshes pointing to `.stl` files
- Meshes are resolved by **filename only** (e.g. `package://foo/bar/pelvis.STL` matches `pelvis.stl`)
- Binary STL format only
- URDF and meshes can be in subdirectories — just drag the top-level folder

## Coordinate Convention

- NPZ data uses **Z-up** (Isaac Sim / IsaacLab convention)
- The viewer converts to Y-up for rendering automatically
- Quaternions are **wxyz** (scalar first)

## Deployment

Single HTML file, no build step needed.

**GitHub Pages:** Fork this repo → Settings → Pages → deploy from `main` branch.

**Local:** Open `index.html` directly in a browser, or serve with any static server:

```bash
python -m http.server 8000
```

## License

[MIT](LICENSE)