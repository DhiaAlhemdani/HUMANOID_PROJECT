#!/usr/bin/env python3
"""Where did each foot land on the terrain? One row per touchdown of a run_case.sh case on a TERRAIN scene.

usage: terrain_strides.py <case_dir> [<case_dir> ...] [--scene scene.xml] [--edge-mm 4] [--csv out.csv]

The case's scene.xml (written by run_case.sh) is loaded with mujoco and the ground under the landing foot is
ray-cast at the four sole corners, so every touchdown gets:
  t, vx     sim time of the LF/RF -> STANCE switch and the commanded speed there
  foot      landing side
  x, y      true world position of the foot's contact frame (ground-truth base + estimated base-to-foot vector)
  gnd_cm    ground height under the foot centre
  step_cm   gnd under this foot minus gnd under the other (stance) foot: the terrain step of this stride
  edge_cm   max - min of the ground height at the four sole corners: > --edge-mm means the foot straddles an
            edge and rests on heel or toe only (the partial-contact landing). On a ramp this is the ramp's own
            rise under the sole (0.17 m x tan 10 deg = 3.0 cm) whatever the foot does: read `tilt` there.
  v         measured base velocity along the heading at the switch [m/s] (the slope foot reference's
            `velocityAlong`: below `standingSpeed` 0.05 it plans every footfall at the feet's midpoint height)
  base      base x ahead of the feet's midpoint along the heading at the switch [cm] (+ = body in front of
            the feet in body terms; on an incline the stance can only hold this inside the sole)
  tdref     planned rise of the landing foot over the stance foot (solver `terrain/next_touchdown` minus the
            stance foot's height at the last solve before the switch, estimator frame) minus the true rise
            (`step`, ground truth) [cm]: negative = the reference was below where the foot landed (it met the
            ground early), positive = above (it landed late). Blank when that solve ran the flat branch (no
            plane: `terrain/next_touchdown` is nan there) - on an incline a blank IS the finding (Run 25 C)
  tilt_deg  sole pitch relative to the ground under it, 0.2 s after the switch (the foot settled): the true
            base pose (ground truth) and the measured joint angles posed in the scene, the ankle-roll link's x
            axis against the ground's rise from heel to toe corners; positive = toe up (resting on the heel),
            negative = toe down (on the toe). Needs meas_q/* in the tracking stream and the robot in scene.xml.
  pitch_pp  true base pitch excursion over the 0.4 s after the switch [deg]
  dz_pp     true base height excursion over the same window [cm]
  FELL      the run's fall lies within 1.5 s after this switch
and a summary: how many landings were edge landings, step-up and step-down, the pitch excursion after each
class, and the class of the stride(s) before the fall. Needs mujoco (e.g. ~/.venvs/replay/bin/python).
Sole corners: --heel / --toe along the foot x axis from the contact frame, --half-width across (G1 collision
spheres sit at -0.05 and +0.12 from the ankle; the contact frame is the sole centre, so +-0.085 by default).
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

try:
    import mujoco
except ImportError:  # pragma: no cover - reported at runtime
    mujoco = None

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("cases", nargs="+")
p.add_argument("--scene", help="scene.xml to ray-cast (default <case>/scene.xml)")
p.add_argument("--edge-mm", type=float, default=4.0)
p.add_argument("--heel", type=float, default=-0.085)
p.add_argument("--toe", type=float, default=0.085)
p.add_argument("--half-width", type=float, default=0.027)
p.add_argument("--after", type=float, default=0.4)
p.add_argument("--csv")
a = p.parse_args()

if mujoco is None:
    sys.exit("terrain_strides.py needs mujoco (pip install mujoco, or ~/.venvs/replay/bin/python)")


def side(name):
    n = name.lower()
    return "l" if ("left" in n or "_l_" in n or n.endswith("_l") or n.startswith("l_")) else "r"


class Ground:
    def __init__(self, scene):
        self.m = mujoco.MjModel.from_xml_path(scene)
        self.d = mujoco.MjData(self.m)
        if self.m.nq >= 3:  # park the robot far away so rays only meet the terrain
            self.d.qpos[:3] = [0.0, 100.0, 5.0]
        mujoco.mj_forward(self.m, self.d)
        self.geomid = np.zeros(1, dtype=np.int32)
        free = [i for i in range(self.m.njnt) if self.m.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE]
        self.base_qadr = self.m.jnt_qposadr[free[0]] if len(free) == 1 else None
        self.pose = mujoco.MjData(self.m)

    def sole_pitch(self, base, joints, body):
        """World pitch of the ankle-roll link's x axis [rad], toe up positive; nan without a posable robot."""
        bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, body)
        if self.base_qadr is None or bid < 0:
            return math.nan
        x, y, z, roll, pitch, yaw = base
        cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2),
                                  math.cos(yaw / 2), math.sin(yaw / 2))
        self.pose.qpos[self.base_qadr:self.base_qadr + 3] = [x, y, z]
        self.pose.qpos[self.base_qadr + 3:self.base_qadr + 7] = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                                                                cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
        for name, q in joints:
            jid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self.pose.qpos[self.m.jnt_qposadr[jid]] = q
        mujoco.mj_kinematics(self.m, self.pose)
        ex = self.pose.xmat[bid].reshape(3, 3)[:, 0]  # the link's x axis in the world
        return math.asin(max(-1.0, min(1.0, float(ex[2]))))

    def z(self, x, y):
        dist = mujoco.mj_ray(self.m, self.d, np.array([x, y, 5.0]), np.array([0.0, 0.0, -1.0]), None, 1, -1, self.geomid)
        return 5.0 - dist if dist >= 0 else math.nan


rows_all = []
for case in a.cases:
    npz = os.path.join(case, "data.npz") if os.path.isdir(case) else case
    cdir = case if os.path.isdir(case) else os.path.dirname(case)
    tag = os.path.basename(cdir.rstrip("/"))
    scene = a.scene or os.path.join(cdir, "scene.xml")
    if not os.path.isfile(scene):
        print(f"{tag}: no scene.xml ({scene})", file=sys.stderr)
        continue
    ground = Ground(scene)
    d = np.load(npz, allow_pickle=False)
    TN = list(d["track_names"]) if "track_names" in d.files else []
    if not TN or not d["track"].size:
        print(f"{tag}: no tracking stream", file=sys.stderr)
        continue
    TR = d["track"]
    col = {n: i for i, n in enumerate(TN)}
    t = TR[:, col["t"]]
    o = np.argsort(t)
    TR, t = TR[o], t[o]
    keep = np.concatenate([[True], np.diff(t) > 1e-6])
    TR, t = TR[keep], t[keep]
    sig = lambda n: TR[:, col[n]] if n in col else None
    gt = d["gt"]
    gt = gt[np.argsort(gt[:, 0])]
    gt = gt[np.concatenate([[True], np.diff(gt[:, 0]) > 1e-6])]
    g = {k: np.interp(t, gt[:, 0], gt[:, i]) for k, i in (("x", 2), ("y", 3), ("z", 4), ("roll", 8), ("pitch", 9), ("yaw", 10))}
    fell = float(d["fell"]) if "fell" in d.files and np.isfinite(float(d["fell"])) else None
    fell_sim = float(np.interp(fell, gt[:, 1], gt[:, 0])) if fell is not None else None
    K = d["script"] if "script" in d.files and d["script"].size else None
    SETTLE = float(d["phases"][0])
    t_wall = np.interp(t, gt[:, 0], gt[:, 1])
    vx_cmd = np.interp(t_wall - SETTLE, K[:, 0], K[:, 1]) if K is not None else np.full_like(t, np.nan)

    mode = sig("plan_mode")
    mode = np.where(np.isfinite(mode), mode, -1.0)
    feet = sorted({n.split("/")[1] for n in TN if n.startswith("obs_foot/")})
    by_side = {side(f): f for f in feet}
    swinging = {1: "l", 2: "r"}  # RF (1) = right in contact, left swings
    ob = {ax: sig(f"obs_base/{ax}") for ax in ("x", "y")}
    meas_q = [(n.split("/", 1)[1], col[n]) for n in TN if n.startswith("meas_q/")]
    ankle_body = {s: f"{'left' if s == 'l' else 'right'}_ankle_roll_link" for s in ("l", "r")}
    n_settle = int(round(0.2 / float(np.median(np.diff(t)))))
    ov = {ax: sig(f"obs_base/{ax}") for ax in ("vx", "vy")}
    SN = [str(n) for n in d["solver_names"]] if "solver_names" in d.files else []
    SV = d["solver"] if SN and "solver" in d.files and d["solver"].size else None
    sc = lambda n: SV[:, SN.index(n)] if SV is not None and n in SN else None
    st, td_next = sc("t"), sc("terrain/next_touchdown")
    if st is not None:
        so = np.argsort(st)
        st = st[so]
        td_next = td_next[so] if td_next is not None else None

    def foot_world(f, k):
        fx, fy = sig(f"obs_foot/{f}/x")[k], sig(f"obs_foot/{f}/y")[k]
        if ob["x"] is not None and ob["y"] is not None:
            return g["x"][k] + fx - ob["x"][k], g["y"][k] + fy - ob["y"][k]
        return fx, fy  # no estimated base in the stream: trust the estimated odom frame

    def corners(x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(a.heel, -a.half_width), (a.heel, a.half_width), (a.toe, -a.half_width), (a.toe, a.half_width)]
        return [ground.z(x + c * u - s * v, y + s * u + c * v) for u, v in pts]

    dt = float(np.median(np.diff(t)))
    n_after = int(round(a.after / dt))
    rows = []
    sw = np.flatnonzero(np.diff(mode) != 0) + 1
    for k in sw:
        m0, m1 = int(mode[k - 1]), int(mode[k])
        if m0 not in swinging or m1 != 3:
            continue
        s = swinging[m0]
        f, other = by_side.get(s), by_side.get("r" if s == "l" else "l")
        if f is None or other is None:
            continue
        x, y = foot_world(f, k)
        ox, oy = foot_world(other, k)
        cz = corners(x, y, g["yaw"][k])
        gz = ground.z(x, y)
        oz = ground.z(ox, oy)
        k1 = min(len(t) - 1, k + n_after)
        # Sole tilt relative to the ground 0.2 s after the switch (the foot has settled): world sole pitch from
        # the posed robot minus the ground's pitch from heel to toe under it.
        tilt = math.nan
        ks = min(len(t) - 1, k + n_settle)
        if meas_q and np.all(np.isfinite(cz)):
            xs, ys = foot_world(f, ks)
            czs = corners(xs, ys, g["yaw"][ks])
            heel_z, toe_z = np.nanmean(czs[:2]), np.nanmean(czs[2:])
            ground_pitch = math.atan2(toe_z - heel_z, a.toe - a.heel)
            # True base pose, but the ground-truth x/y shifted by the foot's own estimate error (as foot_world).
            base = (g["x"][ks], g["y"][ks], g["z"][ks], g["roll"][ks], g["pitch"][ks], g["yaw"][ks])
            sole = ground.sole_pitch(base, [(n, TR[ks, c]) for n, c in meas_q], ankle_body[s])
            tilt = math.degrees(sole - ground_pitch) if np.isfinite(sole) and np.isfinite(ground_pitch) else math.nan
        pitch_pp = math.degrees(np.ptp(g["pitch"][k:k1 + 1]))
        dz_pp = 100 * np.ptp(g["z"][k:k1 + 1])
        hx, hy = math.cos(g["yaw"][k]), math.sin(g["yaw"][k])
        v_along = ov["vx"][k] * hx + ov["vy"][k] * hy if ov["vx"] is not None and ov["vy"] is not None else math.nan
        base_ahead = math.nan
        if ob["x"] is not None and ob["y"] is not None:
            mx = 0.5 * (sig(f"obs_foot/{f}/x")[k] + sig(f"obs_foot/{other}/x")[k])
            my = 0.5 * (sig(f"obs_foot/{f}/y")[k] + sig(f"obs_foot/{other}/y")[k])
            base_ahead = 100 * ((ob["x"][k] - mx) * hx + (ob["y"][k] - my) * hy)
        td_err = math.nan
        oz_est = sig(f"obs_foot/{other}/z")
        if td_next is not None and oz_est is not None:
            j = int(np.searchsorted(st, t[k])) - 1
            if j >= 0 and np.isfinite(td_next[j]) and t[k] - st[j] < 0.2:
                # Planned rise over the stance foot at that solve, against the true rise: the estimator frame
                # re-references at touchdown, so planned-minus-landed heights would carry that shift.
                planned_rise = td_next[j] - float(np.interp(st[j], t, oz_est))
                td_err = 100 * planned_rise - 100 * (gz - oz)
        fell_here = fell_sim is not None and 0 <= fell_sim - t[k] <= 1.5
        rows.append(dict(case=tag, t=t[k], vx=vx_cmd[k], foot=s, x=x, y=y, gnd_cm=100 * gz, step_cm=100 * (gz - oz),
                         edge_cm=100 * (np.nanmax(cz) - np.nanmin(cz)), v=v_along, base_cm=base_ahead, tdref_cm=td_err,
                         tilt_deg=tilt, pitch_pp=pitch_pp, dz_pp=dz_pp, FELL=fell_here))
    if not rows:
        print(f"{tag}: no touchdowns", file=sys.stderr)
        continue
    rows_all += rows

    print(f"\n######## {tag}  ({scene})")
    if fell_sim is not None:
        kf = int(np.searchsorted(t, fell_sim))
        kf = min(kf, len(t) - 1)
        print(f"  FELL at sim {fell_sim:.2f} s, base at x {g['x'][kf]:.2f} y {g['y'][kf]:.2f} (vx cmd {vx_cmd[kf]:.1f})")
    hdr = (f"{'t':>7} {'vx':>4} {'ft':>2} {'x':>6} {'y':>6} {'gnd':>5} {'step':>5} {'edge':>5} {'v':>5} {'base':>5} {'tdref':>5} "
           f"{'tilt':>5} {'pitch':>6} {'dz':>5}  ")
    print("  " + hdr)
    for r in rows:
        print(f"  {r['t']:7.2f} {r['vx']:4.1f} {r['foot']:>2} {r['x']:6.2f} {r['y']:6.2f} {r['gnd_cm']:5.1f} {r['step_cm']:+5.1f} "
              f"{r['edge_cm']:5.1f} {r['v']:5.2f} {r['base_cm']:5.1f} {r['tdref_cm']:5.1f} {r['tilt_deg']:5.1f} "
              f"{r['pitch_pp']:6.2f} {r['dz_pp']:5.1f}  {'FELL' if r['FELL'] else ''}")

    R = np.array([(r["step_cm"], r["edge_cm"], r["pitch_pp"], r["dz_pp"], r["FELL"]) for r in rows], dtype=float)
    edge = R[:, 1] > a.edge_mm / 10
    up, down = R[:, 0] > 0.5, R[:, 0] < -0.5
    walking = np.isfinite(R[:, 2]) & (R[:, 4] == 0)  # class means over the strides that did not end in the fall

    def mean(mask, c):
        v = R[mask & walking, c]
        return f"{np.mean(v):5.2f} (n={mask.sum():3d})" if v.size else "   -  (n=  0)"

    print(f"  summary: {len(rows)} landings | edge (> {a.edge_mm:.0f} mm across the sole) {100 * edge.mean():.0f} % | "
          f"step up {100 * up.mean():.0f} % down {100 * down.mean():.0f} % flat {100 * (~up & ~down).mean():.0f} %")
    print(f"  pitch excursion after landing [deg]: edge {mean(edge, 2)} | no edge {mean(~edge, 2)} | "
          f"up {mean(up, 2)} | down {mean(down, 2)} | flat {mean(~up & ~down, 2)}")
    print(f"  height excursion after landing [cm]:  edge {mean(edge, 3)} | no edge {mean(~edge, 3)} | "
          f"up {mean(up, 3)} | down {mean(down, 3)} | flat {mean(~up & ~down, 3)}")
    tdr = np.array([r["tdref_cm"] for r in rows], dtype=float)
    if np.isfinite(tdr).any():
        ok = np.isfinite(tdr)
        slow = ok & (np.abs(np.array([r["v"] for r in rows], dtype=float)) < 0.05)
        print(f"  planned minus true rise of the landing foot [cm]: mean {np.mean(tdr[ok]):+5.2f} | mean |err| "
              f"{np.mean(np.abs(tdr[ok])):5.2f} | worst {tdr[ok][np.argmax(np.abs(tdr[ok]))]:+5.2f} | "
              + (f"at |v| < 0.05 m/s (n={slow.sum()}): {np.mean(tdr[slow]):+5.2f}" if slow.any() else "no landing at |v| < 0.05 m/s")
              + f" | plane active (tdref present) on {100 * ok.mean():.0f} % of landings")
    elif td_next is not None:
        print("  plane active (tdref present) on 0 % of landings: the flat branch ran at every touchdown")
    tilts = np.array([r["tilt_deg"] for r in rows], dtype=float)
    if np.isfinite(tilts).any():
        ok = np.isfinite(tilts)
        print(f"  sole tilt to the ground 0.2 s after landing [deg]: mean {np.mean(tilts[ok]):+5.2f} | mean |tilt| "
              f"{np.mean(np.abs(tilts[ok])):5.2f} | max |tilt| {np.max(np.abs(tilts[ok])):5.2f} | "
              f"> 3 deg on {100 * np.mean(np.abs(tilts[ok]) > 3):.0f} % of landings")
    big = sorted(rows, key=lambda r: -abs(r["step_cm"]))[:4]
    print("  largest steps: " + "; ".join(
        f"{r['step_cm']:+.1f} cm at t {r['t']:.1f} (vx {r['vx']:.1f}, pitch {r['pitch_pp']:.1f}{', FELL' if r['FELL'] else ''})"
        for r in big))
    if fell_sim is not None:
        last = [r for r in rows if r["FELL"]]
        print("  strides before the fall: " + "; ".join(
            f"{r['foot']} step {r['step_cm']:+.1f} edge {r['edge_cm']:.1f} cm tilt {r['tilt_deg']:+.1f} deg" for r in last[-3:]))

if a.csv and rows_all:
    import csv
    with open(a.csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows_all[0].keys()))
        w.writeheader()
        w.writerows(rows_all)
    print("wrote", a.csv)
