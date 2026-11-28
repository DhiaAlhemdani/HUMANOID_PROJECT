#!/usr/bin/env python3
"""Where does the lateral drift of a walk come from: turning (yaw) or crabbing (side-slip)?

    tests/tuning/lateral_drift.py tuning_results/humanTEV__speeds [more cases] [--bin 1.0]
        [--step-width 0.0] [--csv]

Reads data.npz only (no MuJoCo). For every case it prints
  * a table per `--bin` metres of forward travel: ground-truth yaw [deg], heading of the travelled
    path [deg], the crab angle (heading - yaw), body-frame lateral speed, the number of landings and
    the mean body-frame lateral foothold offset of the left / right foot relative to the pelvis;
  * a landing table (one row per touchdown read off plan_mode): body-frame foothold offset of the
    landing foot, the stance width (left - right lateral offset) and the yaw change since the
    previous landing;
  * totals: final yaw, final y, how much of y is explained by turning (integral of
    |v| sin(yaw)) and how much by crabbing (integral of body-frame vy).
"heading - yaw" near 0 with a growing yaw means the robot turns and walks straight in its own frame
(yaw drift: a yaw-rate / foothold-yaw problem); heading - yaw of several degrees with a flat yaw means
the robot slides sideways (foothold lateral bias or stance-foot slip).
"""
import argparse
import math
import os
import sys

import numpy as np

p = argparse.ArgumentParser()
p.add_argument("cases", nargs="+")
p.add_argument("--bin", type=float, default=1.0, help="forward-travel bin [m]")
p.add_argument("--step-width", type=float, default=0.0,
               help="nominal half stance width [m] subtracted from |foothold offset| (0 = report raw)")
p.add_argument("--csv", action="store_true", help="also write <case>/lateral_drift.csv")
a = p.parse_args()


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def side(frame):
    f = frame.lower()
    return "l" if f.startswith("l") or "left" in f else "r"


for case in a.cases:
    npz = case if case.endswith(".npz") else os.path.join(case, "data.npz")
    tag = os.path.basename(os.path.dirname(npz) if npz.endswith("data.npz") else npz)
    if not os.path.exists(npz):
        print(f"{tag}: missing {npz}", file=sys.stderr)
        continue
    d = np.load(npz, allow_pickle=False)
    gt = d["gt"]
    gt = gt[np.argsort(gt[:, 0])]
    gt = gt[np.concatenate([[True], np.diff(gt[:, 0]) > 1e-6])]
    t = gt[:, 0]
    x, y, yaw = gt[:, 2], gt[:, 3], gt[:, 10]
    yaw_u = np.unwrap(yaw)
    fell = float(d["fell"]) if "fell" in d.files and np.isfinite(float(d["fell"])) else None
    if fell is not None:
        t_end = float(np.interp(fell, gt[:, 1], gt[:, 0]))
        keep = t <= t_end
        t, x, y, yaw_u = t[keep], x[keep], y[keep], yaw_u[keep]
    # World velocities from the ground-truth positions (the odom twist is in the body frame), smoothed over ~50 ms.
    win = max(1, int(round(0.05 / max(float(np.median(np.diff(t))), 1e-3))))
    smooth = lambda v: np.convolve(v, np.ones(win) / win, mode="same") if win > 1 else v
    vx, vy = smooth(np.gradient(x, t)), smooth(np.gradient(y, t))
    # Body-frame velocities from world velocities and ground-truth yaw.
    c, s = np.cos(yaw_u), np.sin(yaw_u)
    vx_b = c * vx + s * vy
    vy_b = -s * vx + c * vy
    dt = np.diff(t, prepend=t[0])
    # Decomposition of y: turning carries the forward speed sideways, crabbing adds body-frame vy.
    y_turn = np.cumsum(vx_b * s * dt)
    y_crab = np.cumsum(vy_b * c * dt)
    direction = 1.0 if (x[-1] - x[0]) >= 0 else -1.0
    forward = direction * (x - x[0])

    # Landings from the tracking stream (plan_mode transitions), footholds in the body frame.
    rows = []
    TN = [str(n) for n in d["track_names"]] if "track_names" in d.files else []
    if TN and d["track"].size:
        TR = d["track"]
        col = {n: i for i, n in enumerate(TN)}
        tt = TR[:, col["t"]]
        o = np.argsort(tt)
        TR, tt = TR[o], tt[o]
        k2 = np.concatenate([[True], np.diff(tt) > 1e-6])
        TR, tt = TR[k2], tt[k2]
        sig = lambda n: TR[:, col[n]] if n in col else None
        mode = sig("plan_mode")
        feet = sorted({n.split("/")[1] for n in TN if n.startswith("obs_foot/")})
        by_side = {side(f): f for f in feet}
        ob = {ax: sig(f"obs_base/{ax}") for ax in ("x", "y", "yaw")}
        g_yaw = np.interp(tt, t, yaw_u)
        g_x = np.interp(tt, t, x)
        # Estimated base pose if the stream has it (same odom frame as the estimated feet), else ground truth.
        base_x = ob["x"] if ob["x"] is not None else g_x
        base_y = ob["y"] if ob["y"] is not None else np.interp(tt, t, y)
        base_yaw = ob["yaw"] if ob["yaw"] is not None else g_yaw
        missing = [n for n, v in (("plan_mode", mode), ("obs_foot (two feet)", by_side if len(by_side) == 2 else None)) if v is None]
        if not missing:
            mode = np.where(np.isfinite(mode), mode, -1.0)
            swinging = {1: "l", 2: "r"}
            last_yaw = None
            for k in np.flatnonzero(np.diff(mode) != 0) + 1:
                m0, m1 = int(mode[k - 1]), int(mode[k])
                if m0 not in swinging or m1 != 3 or (fell is not None and tt[k] > t_end):
                    continue
                sd = swinging[m0]
                off = {}
                for ss, f in by_side.items():
                    dx = sig(f"obs_foot/{f}/x")[k] - base_x[k]
                    dy = sig(f"obs_foot/{f}/y")[k] - base_y[k]
                    yb = base_yaw[k]
                    off[ss] = -math.sin(yb) * dx + math.cos(yb) * dy
                dyaw = 0.0 if last_yaw is None else math.degrees(g_yaw[k] - last_yaw)
                last_yaw = g_yaw[k]
                rows.append((tt[k], direction * (g_x[k] - x[0]), sd, off["l"], off["r"], off["l"] - off["r"], dyaw))

    print(f"=== {tag}: {'FELL at %.1f s' % t_end if fell is not None else 'no fall'}, "
          f"travel {direction * (x[-1] - x[0]):+.2f} m in x, final y {y[-1] - y[0]:+.2f} m, "
          f"final yaw {math.degrees(yaw_u[-1] - yaw_u[0]):+.1f} deg")
    print(f"  y explained by turning {y_turn[-1]:+.2f} m, by crabbing {y_crab[-1]:+.2f} m "
          f"(sum {y_turn[-1] + y_crab[-1]:+.2f}, gt {y[-1] - y[0]:+.2f})")
    print(f"  {'x-bin':>9} {'yaw':>7} {'heading':>8} {'crab':>6} {'vy_body':>8} {'n_land':>6} {'off_L':>7} {'off_R':>7} {'width':>7}")
    edges = np.arange(0.0, forward.max() + a.bin, a.bin)
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (forward >= lo) & (forward < hi) & (np.abs(vx_b) > 0.05)
        if m.sum() < 5:
            continue
        i0, i1 = np.flatnonzero(m)[[0, -1]]
        heading = math.atan2(y[i1] - y[i0], x[i1] - x[i0])  # direction the pelvis actually travelled
        yaw_mid = float(np.mean(yaw_u[m]))
        face = yaw_mid + (0.0 if direction > 0 else math.pi)  # direction of travel the robot intends
        face0 = yaw_u[0] + (0.0 if direction > 0 else math.pi)
        crab = math.degrees(wrap(heading - face))
        lands = [r for r in rows if lo <= r[1] < hi]
        off_l = np.mean([r[3] for r in lands]) if lands else float("nan")
        off_r = np.mean([r[4] for r in lands]) if lands else float("nan")
        print(f"  {lo:4.1f}-{hi:4.1f} {math.degrees(yaw_mid - yaw_u[0]):+7.1f} {math.degrees(wrap(heading - face0)):+8.1f} "
              f"{crab:+6.1f} {float(np.mean(vy_b[m])):+8.3f} {len(lands):6d} {off_l:+7.3f} {off_r:+7.3f} {off_l - off_r:7.3f}")
    if rows:
        print(f"  landings: {len(rows)}; body-frame lateral offset L mean {np.mean([r[3] for r in rows]):+.3f} "
              f"(sd {np.std([r[3] for r in rows]):.3f}), R mean {np.mean([r[4] for r in rows]):+.3f} "
              f"(sd {np.std([r[4] for r in rows]):.3f}); stance width mean {np.mean([r[5] for r in rows]):.3f}; "
              f"yaw per landing mean {np.mean([r[6] for r in rows[1:]]):+.2f} deg, max |{max(abs(r[6]) for r in rows[1:]):.2f}|")
        print(f"  {'t':>6} {'x':>6} {'ft':>2} {'off_L':>7} {'off_R':>7} {'width':>6} {'dyaw':>6}")
        for r in rows:
            print(f"  {r[0]:6.2f} {r[1]:6.2f} {r[2]:>2} {r[3]:+7.3f} {r[4]:+7.3f} {r[5]:6.3f} {r[6]:+6.2f}")
        if a.csv and npz.endswith("data.npz"):
            out = os.path.join(os.path.dirname(npz), "lateral_drift.csv")
            with open(out, "w") as fh:
                fh.write("t,x,foot,off_l,off_r,width,dyaw_deg\n")
                for r in rows:
                    fh.write(f"{r[0]:.3f},{r[1]:.3f},{r[2]},{r[3]:.4f},{r[4]:.4f},{r[5]:.4f},{r[6]:.3f}\n")
            print(f"  wrote {out}")
    else:
        why = f"missing streams {missing}" if TN and d["track"].size and missing else \
            ("no tracking stream" if not (TN and d["track"].size) else "no swing -> stance transitions in plan_mode")
        print(f"  (landing table skipped: {why})")
