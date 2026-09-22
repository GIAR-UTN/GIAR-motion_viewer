#!/usr/bin/env python3
"""
Convert Go2 PKL motions to B2-native PKL motions.

Go2 motions were recorded with root heights appropriate for the Go2's leg length.
When played on B2 (longer legs), the same root height causes foot penetration.

This script uses the EXACT same FK chain that the viewer builds from b2.xml
(parseMJCF + buildFKChain) to compute the correct root Z offset per frame,
so that the lowest foot sits exactly at Z = 0.

Usage:
    python scripts/convert_go2_to_b2.py
    # generates demo/b2_moves/b2_*.pkl from demo/go2_moves/go2_*.pkl
"""
import os
import pickle
import math
from typing import Tuple


# ── B2 FK chain — EXACT match of what buildFKChain produces from b2.xml ─────
# Body BFS order from computeBodyOrder:
#   [0] base_link       (root, no joint)
#   [1] FL_hip          — FL_hip_joint      (DoF 0)
#   [2] FR_hip          — FR_hip_joint      (DoF 1)
#   [3] RL_hip          — RL_hip_joint      (DoF 2)
#   [4] RR_hip          — RR_hip_joint      (DoF 3)
#   [5] FL_thigh        — FL_thigh_joint    (DoF 4)
#   [6] FR_thigh        — FR_thigh_joint    (DoF 5)
#   [7] RL_thigh        — RL_thigh_joint    (DoF 6)
#   [8] RR_thigh        — RR_thigh_joint    (DoF 7)
#   [9] FL_calf         — FL_calf_joint     (DoF 8)
#  [10] FR_calf         — FR_calf_joint     (DoF 9)
#  [11] RL_calf         — RL_calf_joint     (DoF 10)
#  [12] RR_calf         — RR_calf_joint     (DoF 11)
#
# Chain entries match buildFKChain output:
#   chain[child_idx] = {parent, x, y, z, ax, ay, az, dofIdx, name, type}
B2_CHAIN = [
    None,  # base_link — root
    # FL leg (body index 1)
    {"parent": 0, "x": 0.3285,    "y": 0.072,    "z": 0.0,
     "ax": 1.0, "ay": 0.0, "az": 0.0, "dofIdx": 0, "name": "FL_hip_joint"},
    # FR leg (body index 2)
    {"parent": 0, "x": 0.3285,    "y": -0.072,   "z": 0.0,
     "ax": 1.0, "ay": 0.0, "az": 0.0, "dofIdx": 1, "name": "FR_hip_joint"},
    # RL leg (body index 3)
    {"parent": 0, "x": -0.3285,   "y": 0.072,    "z": 0.0,
     "ax": 1.0, "ay": 0.0, "az": 0.0, "dofIdx": 2, "name": "RL_hip_joint"},
    # RR leg (body index 4)
    {"parent": 0, "x": -0.3285,   "y": -0.072,   "z": 0.0,
     "ax": 1.0, "ay": 0.0, "az": 0.0, "dofIdx": 3, "name": "RR_hip_joint"},
    # FL thigh (body index 5)
    {"parent": 1, "x": 0.0,       "y": 0.11973,  "z": 0.0,
     "ax": 0.0, "ay": 1.0, "az": 0.0, "dofIdx": 4, "name": "FL_thigh_joint"},
    # FR thigh (body index 6)
    {"parent": 2, "x": 0.0,       "y": -0.11973, "z": 0.0,
     "ax": 0.0, "ay": 1.0, "az": 0.0, "dofIdx": 5, "name": "FR_thigh_joint"},
    # RL thigh (body index 7)
    {"parent": 3, "x": 0.0,       "y": 0.11973,  "z": 0.0,
     "ax": 0.0, "ay": 1.0, "az": 0.0, "dofIdx": 6, "name": "RL_thigh_joint"},
    # RR thigh (body index 8)
    {"parent": 4, "x": 0.0,       "y": -0.11973, "z": 0.0,
     "ax": 0.0, "ay": 1.0, "az": 0.0, "dofIdx": 7, "name": "RR_thigh_joint"},
    # FL calf (body index 9)
    {"parent": 5, "x": 0.0,       "y": -8.6984e-05, "z": -0.35,
     "ax": 0.0, "ay": 1.0, "az": 0.0, "dofIdx": 8, "name": "FL_calf_joint"},
    # FR calf (body index 10)
    {"parent": 6, "x": 0.0,       "y": 8.6986e-05,  "z": -0.35,
     "ax": 0.0, "ay": 1.0, "az": 0.0, "dofIdx": 9, "name": "FR_calf_joint"},
    # RL calf (body index 11)
    {"parent": 7, "x": 0.0,       "y": -8.6984e-05, "z": -0.35,
     "ax": 0.0, "ay": 1.0, "az": 0.0, "dofIdx": 10, "name": "RL_calf_joint"},
    # RR calf (body index 12)
    {"parent": 8, "x": 0.0,       "y": 8.6986e-05,  "z": -0.35,
     "ax": 0.0, "ay": 1.0, "az": 0.0, "dofIdx": 11, "name": "RR_calf_joint"},
]
N_BODIES = len(B2_CHAIN)  # 13
FOOT_BODIES = [9, 10, 11, 12]  # FL_calf, FR_calf, RL_calf, RR_calf


# ── Quaternion helpers ────────────────────────────────────────────────────────
def expmap_to_quat(ex: float, ey: float, ez: float) -> Tuple[float, float, float, float]:
    """Convert exponential map (axis * angle) to unit quaternion (w, x, y, z)."""
    angle = math.sqrt(ex * ex + ey * ey + ez * ez)
    if angle < 1e-8:
        return (1.0, 0.0, 0.0, 0.0)
    s = 0.5 if angle < 1e-12 else math.sin(angle / 2) / angle
    return (math.cos(angle / 2), ex * s, ey * s, ez * s)


# ── Forward Kinematics (exact replica of viewer's precomputeFK) ──────────────
def fk(chain, root_pos, root_quat, dof_pos, n_bodies, n_dof):
    """
    Compute body positions using the viewer's exact FK algorithm.
    Returns a list of Z coordinates for all bodies.
    """
    bp = [0.0] * (n_bodies * 3)
    bq = [0.0] * (n_bodies * 4)

    bp[0] = root_pos[0]
    bp[1] = root_pos[1]
    bp[2] = root_pos[2]
    bq[0] = root_quat[0]
    bq[1] = root_quat[1]
    bq[2] = root_quat[2]
    bq[3] = root_quat[3]

    for i in range(1, n_bodies):
        c = chain[i]
        if c is None:
            bp[i * 3] = bp[i * 3 + 1] = bp[i * 3 + 2] = 0.0
            bq[i * 4] = 1.0
            bq[i * 4 + 1] = bq[i * 4 + 2] = bq[i * 4 + 3] = 0.0
            continue

        pi = c["parent"]
        a = dof_pos[c["dofIdx"]] if 0 <= c["dofIdx"] < n_dof else 0.0

        pw = bq[pi * 4]
        px = bq[pi * 4 + 1]
        py = bq[pi * 4 + 2]
        pz = bq[pi * 4 + 3]

        vx, vy, vz = c["x"], c["y"], c["z"]
        tx = 2.0 * (py * vz - pz * vy)
        ty = 2.0 * (pz * vx - px * vz)
        tz = 2.0 * (px * vy - py * vx)

        bp[i * 3]     = bp[pi * 3]     + vx + pw * tx + py * tz - pz * ty
        bp[i * 3 + 1] = bp[pi * 3 + 1] + vy + pw * ty + pz * tx - px * tz
        bp[i * 3 + 2] = bp[pi * 3 + 2] + vz + pw * tz + px * ty - py * tx

        # Joint rotation quaternion
        h = a * 0.5
        sh = math.sin(h)
        ch = math.cos(h)
        qaw = ch
        qax = c["ax"] * sh
        qay = c["ay"] * sh
        qaz = c["az"] * sh
        qow = c.get("qow", 1.0)
        qox = c.get("qox", 0.0)
        qoy = c.get("qoy", 0.0)
        qoz = c.get("qoz", 0.0)

        qlw = qow * qaw - qox * qax - qoy * qay - qoz * qaz
        qlx = qow * qax + qox * qaw + qoy * qaz - qoz * qay
        qly = qow * qay - qox * qaz + qoy * qaw + qoz * qax
        qlz = qow * qaz + qox * qay - qoy * qax + qoz * qaw

        bq[i * 4]     = pw * qlw - px * qlx - py * qly - pz * qlz
        bq[i * 4 + 1] = pw * qlx + px * qlw + py * qlz - pz * qly
        bq[i * 4 + 2] = pw * qly - px * qlz + py * qlw + pz * qlx
        bq[i * 4 + 3] = pw * qlz + px * qly - py * qlx + pz * qlw

    return bp


def get_min_foot_z(root_pos, root_quat, dof_pos):
    """Return the minimum Z among the four foot bodies."""
    bp = fk(B2_CHAIN, root_pos, root_quat, dof_pos, N_BODIES, 12)
    return min(bp[i * 3 + 2] for i in FOOT_BODIES)


def convert_frame(root_x, root_y, root_z, ex, ey, ez, joints):
    """
    Convert one Go2 frame to a B2 frame.

    The offset is computed so the lowest B2 foot lands exactly at Z=0.
    """
    root_quat = expmap_to_quat(ex, ey, ez)
    min_fz = get_min_foot_z((root_x, root_y, root_z), root_quat, joints)
    delta_z = -min_fz  # positive = raise root to lift feet off ground
    return root_z + delta_z, delta_z


def convert_motion(src_path, dst_path):
    """Read a Go2 PKL motion and write a B2-native PKL motion."""
    with open(src_path, 'rb') as f:
        data = pickle.load(f, encoding='latin1')

    fps = data['fps']
    frames = data['frames']
    loop_mode = data.get('loop_mode', 0)
    n_frames = len(frames)

    print(f"  {os.path.basename(src_path)}: {n_frames} frames, fps={fps}")

    # Compute per-frame offset and find the maximum
    offsets = []
    max_penetration = 0.0
    for fr in frames:
        root_quat = expmap_to_quat(fr[3], fr[4], fr[5])
        min_fz = get_min_foot_z((fr[0], fr[1], fr[2]), root_quat, fr[6:])
        pen = -min_fz  # positive = penetration depth
        offsets.append(pen)
        if pen > max_penetration:
            max_penetration = pen

    # Use a uniform offset = max penetration (raises all frames equally)
    offset = max_penetration
    new_frames = []
    for i, fr in enumerate(frames):
        new_z = fr[2] + offset
        new_frames.append([fr[0], fr[1], new_z, fr[3], fr[4], fr[5]] + list(fr[6:]))

    # Verify
    worst_after = 0.0
    worst_frame = 0
    for fi, fr in enumerate(new_frames):
        root_quat = expmap_to_quat(fr[3], fr[4], fr[5])
        mz = get_min_foot_z((fr[0], fr[1], fr[2]), root_quat, fr[6:])
        if mz < worst_after:
            worst_after = mz
            worst_frame = fi

    print(f"    Max penetration (before): {max_penetration:.4f}m")
    print(f"    Offset applied: +{offset:.4f}m")
    print(f"    Worst foot Z after: {worst_after:.6f}m (frame {worst_frame})")

    new_data = {'fps': fps, 'frames': new_frames, 'loop_mode': loop_mode}
    with open(dst_path, 'wb') as f:
        pickle.dump(new_data, f, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"    Written: {dst_path}")


def main():
    src_dir = 'demo/go2_moves'
    dst_dir = 'demo/b2_moves'
    os.makedirs(dst_dir, exist_ok=True)

    pkl_files = sorted(f for f in os.listdir(src_dir) if f.endswith('.pkl'))
    if not pkl_files:
        print("No PKL files found in demo/go2_moves/")
        return

    print(f"Converting {len(pkl_files)} Go2 motion(s) to B2-native format.\n")

    for fname in pkl_files:
        src = os.path.join(src_dir, fname)
        dst_fname = fname.replace('go2_', 'b2_')
        dst = os.path.join(dst_dir, dst_fname)
        try:
            convert_motion(src, dst)
        except Exception as e:
            import traceback
            print(f"  ERROR converting {fname}: {e}")
            traceback.print_exc()
        print()

    print("Done. B2 motions are in demo/b2_moves/")


if __name__ == '__main__':
    main()
