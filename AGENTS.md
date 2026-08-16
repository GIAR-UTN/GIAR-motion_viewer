# AGENTS.md

Browser-based humanoid motion viewer/player (GIAR fork of motion-viewer). **All application code lives in a single file, `index.html` (~2300 lines of vanilla JS + Three.js via CDN).** There is no build step, no `package.json`, no test suite, no linter, no typecheck — do not look for or run any of those.

## How to verify changes
- No automated verification exists. Test manually: `python -m http.server 8000` in the repo root, open `http://localhost:8000`, then drag `demo/unitree_g1/` + `demo/dance1_subject2.npz` into the window.
- Editing `index.html` does not require a rebuild; just reload the page.

## Structure
- `index.html` — the entire app: URDF/MJCF parsers, NPZ/PKL/BVH loaders, forward kinematics, Three.js rendering, UI.
- `demo/` — sample robot + motion. Note `demo/dance1_subject2.npz` is only 133 bytes (a stub), not the full 131s dance described in the README.
- `unitree_ros/` — **vendored subset** of Unitree ROS descriptions, tracked directly in this repo. It was pruned from the full `unitree_ros` clone to only the description files + STL meshes used by `MODEL_LIBRARY` (~232 MB, 378 files). It is NOT a nested git repo anymore (the nested `.git` was removed) — treat it as plain repo content.
- Static site files: `CNAME`, `sitemap.xml`, `robots.txt`, `google*.html` (site verification) — served on GitHub Pages.

## Binary assets (not Git LFS)
STL, npz, and `.gitattributes` mark these as **binary, committed directly as regular git blobs** — Git LFS is NOT used in this repo (not installed). This matches the existing `demo/unitree_g1/*.STL` and `demo/dance1_subject2.npz`. Do not try to route STL/npz through LFS; just `git add` them.

## Data conventions (easy to get wrong)
- Motion coordinates are **Z-up** (Isaac/MuJoCo); the viewer converts to Y-up for rendering — do not "fix" the raw data.
- NPZ quaternions are **wxyz**; PKL `root_rot` is **xyzw**.
- NPZ uses keys `fps`, `body_pos_w`, `body_quat_w`; PKL uses `fps`, `root_pos`, `root_rot`, `dof_pos`.
- Mesh resolution is by **filename only** (case-insensitive), binary STL only.
- PKL joint mapping: see `PKL_JOINT_PRESETS` in `index.html` (~line 1071) — DoF slots are matched to URDF joints by name because PKL/MJCF joint sets can differ from the URDF.
- NPZ supports both uncompressed and `deflate-raw`-compressed payloads (via `DecompressionStream`).

## Key parsers in `index.html`
`parseURDF` (~260), `parseMJCF` (~350), `parseBVH` (~503), `parseNPZ` (~797), `parsePickle` (~877), `parseSTL` (~1170), `loadRobot` (~1403), `loadNPZ`/`loadPKL` (~1936/2024). Keep changes inside `index.html`; there is no separate source tree to sync.

## Model library (sidebar "Robot library" select)
- `MODEL_LIBRARY` in `index.html` (~2100) is a **hand-curated** list of robots from `unitree_ros/robots/*` that are STL-viewable. Each entry is `{label, desc, meshDir}` where `desc`/`meshDir` are repo-root-relative paths fetched via `fetch()`.
- Adding a model requires **fetch-path correctness**: every `.stl` basename referenced by `desc` must exist directly in `meshDir` (case-sensitive), because the loader fetches `meshDir/<basename>` and the viewer matches meshes by lowercase basename only.
- Excluded from the library: robots whose description references non-STL meshes (`.obj`/`.dae`) — e.g. `b2_description_mujoco` (OBJ), `h2_plus` (DAE + duplicate basenames across `left_sharpa`/`right_sharpa` which collide in the flat basename map). `z1_description` uses `.dae` visuals.
- The library fetches `unitree_ros/robots/*` over HTTP. Because `unitree_ros/` is now tracked in the repo, the model picker works both locally (`python -m http.server`) and on GitHub Pages without users supplying robots. When adding a model, add its desc + referenced STL meshes to `unitree_ros/` (pruned to only what the loader fetches).
