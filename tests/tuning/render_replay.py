#!/usr/bin/env python3
"""Render a recorded run to video by replaying it kinematically in MuJoCo (no physics): the recorded joint
angles (joint_states / ~/tracking meas_q) drive the joints and the ground-truth odometry drives the base.
A side camera follows the pelvis. For judging smoothness by eye, side by side with the numbers.

usage: render_replay.py <case_dir> <out.mp4> [--fps 50] [--speed 1.0] [--start S --end S (script clock)]
                        [--view side|front|iso] [--width 960 --height 540] [--dry-run]
needs: pip install mujoco imageio imageio-ffmpeg ; a GL backend (MUJOCO_GL=egl on a GPU machine, osmesa
without one, glfw with a display). --dry-run checks the joint mapping and frame count without rendering.
The scene is <case_dir>/scene.xml (what the run used); its robot includes resolve relative to the file.
An overlay prints script time, the command (from data.npz script knots) and the gait mode if recorded.
"""
import argparse
import os
import sys

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("case"); ap.add_argument("out")
    ap.add_argument("--fps", type=float, default=50.0); ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--start", type=float, default=None); ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--view", default="side", choices=["side", "front", "iso"])
    ap.add_argument("--width", type=int, default=960); ap.add_argument("--height", type=int, default=540)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    d = np.load(os.path.join(a.case, "data.npz"), allow_pickle=False)
    gt = d["gt"]                                      # t_sim, t_wall, x, y, z, vx, vy, vz, roll, pitch, yaw
    SETTLE = float(d["phases"][0])
    w2s = lambda w: float(np.interp(w, gt[:, 1], gt[:, 0]))
    t_script0 = w2s(SETTLE)
    # Joint angles: ~/tracking meas_q at the control rate when present, else /joint_states.
    TN = list(d["track_names"]) if "track_names" in d.files else []
    if TN and d["track"].size:
        jn = [n.split("/", 1)[1] for n in TN if n.startswith("meas_q/")]
        jt = d["track"][:, TN.index("t")]
        JQ = np.column_stack([d["track"][:, TN.index("meas_q/" + n)] for n in jn])
        pm = d["track"][:, TN.index("plan_mode")] if "plan_mode" in TN else None
    else:
        jt, jn, JQ, pm = d["js_t"], list(d["js_names"]), d["js_q"], None
    o = np.argsort(jt); jt, JQ = jt[o], JQ[o]
    if pm is not None:
        pm = pm[o]
    K = d["script"] if "script" in d.files and d["script"].size else None

    import mujoco
    scene = os.path.join(a.case, "scene.xml")
    m = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(m)
    free = [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE]
    if len(free) != 1:
        sys.exit(f"{scene}: expected one free joint, found {len(free)}")
    base_qadr = m.jnt_qposadr[free[0]]
    jmap = []
    missing = []
    for k, n in enumerate(jn):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        if jid < 0:
            missing.append(n); continue
        jmap.append((k, m.jnt_qposadr[jid]))
    t0 = max(gt[0, 0], jt[0]) if a.start is None else t_script0 + a.start
    t1 = min(gt[-1, 0], jt[-1]) if a.end is None else t_script0 + a.end
    frames_t = np.arange(t0, t1, a.speed / a.fps)
    print(f"{a.case}: {len(jn)} recorded joints, {len(jmap)} mapped, missing {missing or 'none'}; "
          f"{len(frames_t)} frames at {a.fps} fps ({(t1 - t0) / a.speed:.1f} s of video), script t0 at sim {t_script0:.2f}")
    if a.dry_run:
        return

    import imageio
    def euler_to_quat(roll, pitch, yaw):  # ZYX convention (yaw about z, then pitch, then roll), as the odometry
        cr, sr, cp, sp, cy, sy = np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2), np.cos(yaw / 2), np.sin(yaw / 2)
        return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])

    renderer = mujoco.Renderer(m, a.height, a.width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = 2.6; cam.elevation = -12
    cam.azimuth = {"side": 90, "front": 180, "iso": 135}[a.view]
    writer = imageio.get_writer(a.out, fps=a.fps, codec="libx264", quality=7, macro_block_size=1)
    for i, t in enumerate(frames_t):
        q = np.array([np.interp(t, jt, JQ[:, k]) for k in range(JQ.shape[1])])
        for k, adr in jmap:
            data.qpos[adr] = q[k]
        g = np.array([np.interp(t, gt[:, 0], gt[:, c]) for c in range(2, 11)])
        data.qpos[base_qadr:base_qadr + 3] = g[0:3]
        data.qpos[base_qadr + 3:base_qadr + 7] = euler_to_quat(g[6], g[7], g[8])
        mujoco.mj_forward(m, data)
        cam.lookat[:] = [g[0], g[1], 0.6]
        if a.view == "side":
            cam.azimuth = 90 + np.degrees(g[8])  # keep the side view relative to the heading
        renderer.update_scene(data, camera=cam)
        img = renderer.render().copy()
        ts = t - t_script0
        lines = [f"{os.path.basename(a.case.rstrip('/'))}  t={ts:6.2f} s"]
        if K is not None:
            cmd = [float(np.interp(ts, K[:, 0], K[:, c])) for c in (1, 2, 3)]
            lines.append(f"cmd vx {cmd[0]:+.2f} vy {cmd[1]:+.2f} yaw {cmd[2]:+.2f}")
        if pm is not None:
            mode = {0: "FLY", 1: "RF", 2: "LF", 3: "STANCE"}.get(int(np.interp(t, jt, pm)), "?")
            lines.append(f"plan mode {mode}")
        _overlay(img, lines)
        writer.append_data(img)
        if i % 500 == 0:
            print(f"  frame {i}/{len(frames_t)}")
    writer.close()
    print(f"wrote {a.out}")


def _overlay(img, lines):
    """Tiny built-in text overlay (no font dependency): white boxes with a 5x7 bitmap font would be more work than
    it is worth; use PIL when available, else skip."""
    try:
        from PIL import Image, ImageDraw
        im = Image.fromarray(img); dr = ImageDraw.Draw(im)
        y = 8
        for s in lines:
            dr.rectangle([6, y - 2, 6 + 7 * len(s) + 6, y + 13], fill=(0, 0, 0))
            dr.text((9, y), s, fill=(255, 255, 255)); y += 16
        img[:] = np.asarray(im)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
