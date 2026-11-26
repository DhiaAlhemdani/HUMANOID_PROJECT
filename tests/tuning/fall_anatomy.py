#!/usr/bin/env python3
"""Anatomy of a fall (or of any window) from a run_case.sh data.npz: what went wrong first.

usage: fall_anatomy.py <case_dir | data.npz> [--at T_SIM] [--before 2.5] [--after 0.3] [--step 0.05]
                       [--model description/mujoco/robots/g1/g1_29dof.xml] [--config <controllers.yaml>]

Prints, for the window before the fall (data.npz `fell`, or --at):
  1. first-anomaly table: the sim time at which each signal first leaves its band
     (base pitch / roll 10 deg, base height error 5 cm, foot height error 3 cm, foot slip 0.1 m/s in
     stance, joint tracking error 0.3 rad, torque beyond 90 % of the MJCF actuator range);
  2. a timeline at --step resolution: plan mode, base pitch / roll / z (obs - plan), per-foot
     obs z and contact, the three largest |meas_tau| joints and the three largest tracking errors;
  3. peak torque per joint over the window against its limit;
  4. optional: the arm-vs-body sphere clearance per sample from the controller config
     (constraints.armSelfCollision spheres) if `mujoco` is importable and a model is given — the
     minimum over the window and the time it is reached (a clearance inside minimumDistance + delta
     means the polynomial penalty is active).
Reads only the tracking stream (500 Hz) and the MJCF; works on any case, falls or not.
"""
from __future__ import annotations

import argparse
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("path")
ap.add_argument("--at", type=float, default=None, help="centre of the window [sim s]; default: the recorded fall")
ap.add_argument("--before", type=float, default=2.5)
ap.add_argument("--after", type=float, default=0.3)
ap.add_argument("--step", type=float, default=0.05)
ap.add_argument("--model", default=None, help="MJCF for torque limits and the sphere clearance")
ap.add_argument("--config", default=None, help="controllers yaml (armSelfCollision spheres); default: <case>/config.yaml")
a = ap.parse_args()

P = a.path
case_dir = P if os.path.isdir(P) else os.path.dirname(P)
npz = os.path.join(P, "data.npz") if os.path.isdir(P) else P
d = np.load(npz, allow_pickle=False)
TN = list(d["track_names"]) if "track_names" in d.files else []
TR = d["track"] if TN and d["track"].size else None
if TR is None:
    sys.exit(f"{npz}: no tracking stream (run with diagnostics.trackingRate > 0)")
col = {n: i for i, n in enumerate(TN)}
t = TR[:, col["t"]]
o = np.argsort(t)
TR, t = TR[o], t[o]
keep = np.concatenate([[True], np.diff(t) > 1e-6])
TR, t = TR[keep], t[keep]


def tc(name):
    return TR[:, col[name]] if name in col else None


fell = float(d["fell"]) if "fell" in d.files and np.isfinite(float(d["fell"])) else None
gt = d["gt"] if "gt" in d.files else None
if a.at is not None:
    T = a.at
elif fell is not None:
    # `fell` is on the probe wall clock; gt maps wall -> sim
    T = float(np.interp(fell, gt[:, 1], gt[:, 0])) if gt is not None else fell
else:
    T = float(t[-1])
w = (t >= T - a.before) & (t <= T + a.after)
if w.sum() < 10:
    sys.exit(f"window [{T - a.before:.2f}, {T + a.after:.2f}] has {w.sum()} samples (t in [{t[0]:.2f}, {t[-1]:.2f}])")
tw = t[w]
print(f"== {os.path.basename(case_dir.rstrip('/'))}: window {tw[0]:.2f} .. {tw[-1]:.2f} s sim"
      + (f" (fell at {T:.2f} s sim)" if fell is not None and a.at is None else ""))

joints = [n.split("/", 1)[1] for n in TN if n.startswith("meas_tau/")]
feet = sorted({n.split("/")[1] for n in TN if n.startswith("obs_foot/")})
MODE = {0: "FLY", 1: "RF", 2: "LF", 3: "STANCE"}

# ---- torque limits from the MJCF ------------------------------------------------------------------
limits = {}
model_path = a.model
if model_path is None:
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.join(here, "..", "..", "description", "mujoco", "robots", "g1", "g1_29dof.xml")
    model_path = cand if os.path.exists(cand) else None
if model_path and os.path.exists(model_path):
    try:
        root = ET.parse(model_path).getroot()
        defaults = {}
        for cls in root.iter("default"):
            m = cls.find("motor")
            if m is not None and cls.get("class"):
                defaults[cls.get("class")] = m.get("ctrlrange") or m.get("forcerange")
        for m in root.iter("motor"):
            rng = m.get("ctrlrange") or m.get("forcerange") or defaults.get(m.get("class"), None)
            j = m.get("joint")
            if rng and j:
                lo, hi = (float(x) for x in rng.split())
                limits[j] = max(abs(lo), abs(hi))
        for jn in root.iter("joint"):  # Unitree MJCFs put the limit on the joint
            rng, j = jn.get("actuatorfrcrange"), jn.get("name")
            if rng and j and j not in limits:
                lo, hi = (float(x) for x in rng.split())
                limits[j] = max(abs(lo), abs(hi))
    except Exception as e:  # noqa: BLE001
        print(f"(torque limits: could not parse {model_path}: {e})")

# ---- signals ----------------------------------------------------------------------------------------
def sig(name):
    x = tc(name)
    return x[w] if x is not None else None


pitch, roll = sig("obs_base/pitch"), sig("obs_base/roll")
zo, zp = sig("obs_base/z"), sig("plan_base/z")
mode = sig("plan_mode")
foot_z = {f: (sig(f"obs_foot/{f}/z"), sig(f"plan_foot/{f}/z")) for f in feet}
foot_v = {f: np.hypot(sig(f"obs_foot/{f}/vx"), sig(f"obs_foot/{f}/vy")) if sig(f"obs_foot/{f}/vx") is not None else None for f in feet}
contact = {}
for f in feet:
    cands = [n for n in TN if n.startswith("meas_contact/") and n.endswith("/contact") and f.split("_")[0] in n]
    contact[f] = sig(cands[0]) if cands else None
tau = {j: sig(f"meas_tau/{j}") for j in joints}
err = {j: (sig(f"meas_q/{j}") - sig(f"cmd_q/{j}")) for j in joints if sig(f"cmd_q/{j}") is not None and sig(f"meas_q/{j}") is not None}

# ---- 1. first anomaly -------------------------------------------------------------------------------
def first(cond, label, detail=""):
    idx = np.flatnonzero(cond)
    return (tw[idx[0]], label, detail) if idx.size else None


events = []
if pitch is not None:
    events.append(first(np.abs(pitch) > math.radians(10), "base pitch > 10 deg", f"peak {math.degrees(np.nanmax(np.abs(pitch))):.1f}"))
if roll is not None:
    events.append(first(np.abs(roll) > math.radians(10), "base roll > 10 deg", f"peak {math.degrees(np.nanmax(np.abs(roll))):.1f}"))
if zo is not None and zp is not None:
    events.append(first(np.abs(zo - zp) > 0.05, "base height error > 5 cm", f"peak {100 * np.nanmax(np.abs(zo - zp)):.1f} cm"))
for f in feet:
    zz, zpp = foot_z[f]
    if zz is not None and zpp is not None:
        events.append(first(np.abs(zz - zpp) > 0.03, f"{f} height error > 3 cm", f"peak {100 * np.nanmax(np.abs(zz - zpp)):.1f} cm"))
    if foot_v[f] is not None and mode is not None:
        stance = np.isin(mode, (3, 1 if f.startswith("left") else 2))  # LF = left swings, RF = right swings
        events.append(first((foot_v[f] > 0.1) & stance, f"{f} slip > 0.1 m/s in stance", f"peak {np.nanmax(np.where(stance, foot_v[f], 0)):.2f} m/s"))
for j, e in err.items():
    ev = first(np.abs(e) > 0.3, f"tracking error > 0.3 rad: {j}", f"peak {np.nanmax(np.abs(e)):.2f} rad")
    if ev:
        events.append(ev)
for j, x in tau.items():
    if x is not None and j in limits:
        ev = first(np.abs(x) > 0.9 * limits[j], f"torque > 90 % of limit: {j}", f"peak {np.nanmax(np.abs(x)):.0f} / {limits[j]:.0f} Nm")
        if ev:
            events.append(ev)
events = sorted(e for e in events if e)
print("\n-- first anomalies (sim s | signal | peak in window)")
if not events:
    print("  none of the bands was left in this window")
for tt, label, detail in events[:14]:
    print(f"  {tt:8.2f} | {label:<46} | {detail}")

# ---- 2. timeline --------------------------------------------------------------------------------------
print(f"\n-- timeline (every {a.step:.2f} s): t | mode | pitch roll [deg] | z obs-plan [cm] | feet z [cm] (c=contact) | top |tau| [Nm] | top |q err| [rad]")
times = np.arange(tw[0], tw[-1] + 1e-9, a.step)
for tq in times:
    i = int(np.argmin(np.abs(tw - tq)))
    parts = [f"{tw[i]:7.2f}", MODE.get(int(mode[i]), "?") if mode is not None else "?"]
    parts.append(f"{math.degrees(pitch[i]):6.1f} {math.degrees(roll[i]):6.1f}" if pitch is not None and roll is not None else "   -   -")
    parts.append(f"{100 * (zo[i] - zp[i]):6.1f}" if zo is not None and zp is not None else "   -")
    fz = []
    for f in feet:
        zz = foot_z[f][0]
        c = contact[f]
        fz.append(f"{100 * zz[i]:5.1f}{'c' if c is not None and c[i] > 0.5 else ' '}" if zz is not None else "  -  ")
    parts.append(" ".join(fz))
    top_tau = sorted(((abs(x[i]), j) for j, x in tau.items() if x is not None), reverse=True)[:3]
    parts.append(" ".join(f"{j.replace('_joint', '')[:14]}={v:4.0f}" for v, j in top_tau))
    top_err = sorted(((abs(e[i]), j) for j, e in err.items()), reverse=True)[:3]
    parts.append(" ".join(f"{j.replace('_joint', '')[:14]}={v:4.2f}" for v, j in top_err))
    print(" | ".join(parts))

# ---- 3. torque peaks ------------------------------------------------------------------------------------
print("\n-- peak |meas_tau| per joint in the window (limit from the MJCF if known)")
rows = sorted(((np.nanmax(np.abs(x)), j) for j, x in tau.items() if x is not None), reverse=True)
for v, j in rows:
    lim = limits.get(j)
    print(f"  {j:<28} {v:6.1f}" + (f"  / {lim:5.0f} ({100 * v / lim:3.0f} %)" if lim else ""))

# ---- 4. arm-body clearance (optional) -------------------------------------------------------------------
cfg_path = a.config or os.path.join(case_dir, "config.yaml")
try:
    import mujoco  # noqa: F401
    import yaml

    if not (model_path and os.path.exists(model_path) and os.path.exists(cfg_path)):
        raise RuntimeError("model or config missing")
    cfg = yaml.safe_load(open(cfg_path))
    ros = cfg["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]
    asc = ros["ocs2"]["constraints"]["armSelfCollision"]
    arms = {n: asc["arms"][n] for n in asc["arms"]["names"]}
    bodies = {n: asc["bodies"][n] for n in asc["bodies"]["names"]}
    m = mujoco.MjModel.from_xml_path(model_path)
    dd = mujoco.MjData(m)
    alias = {"left_rubber_hand": "left_wrist_yaw_link", "right_rubber_hand": "right_wrist_yaw_link"}
    body_ok = lambda name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, alias.get(name, name)) >= 0
    arms = {n: s for n, s in arms.items() if body_ok(s["linkName"])}
    bodies = {n: s for n, s in bodies.items() if body_ok(s["linkName"])}
    qcols = {j: col[f"meas_q/{j}"] for j in joints if f"meas_q/{j}" in col}
    jadr = {j: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in qcols
            if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) >= 0}

    def pos(link, c):
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, alias.get(link, link))
        return dd.xpos[b] + dd.xmat[b].reshape(3, 3) @ np.asarray(c, float)

    thr = float(asc.get("minimumDistance", 0.0)) + float(asc.get("delta", 0.0))
    idx = np.flatnonzero(w)[:: max(1, int(round(a.step / 2 / np.median(np.diff(t)))))]
    best = (9.0, None, None)
    inside = 0
    for i in idx:
        mujoco.mj_resetData(m, dd)
        for j, adr in jadr.items():
            dd.qpos[adr] = TR[i, qcols[j]]
        mujoco.mj_kinematics(m, dd)
        dmin, pair = 9.0, None
        for an, s in arms.items():
            pa = pos(s["linkName"], s["center"])
            for bn, b in bodies.items():
                dist = np.linalg.norm(pa - pos(b["linkName"], b["center"])) - s["radius"] - b["radius"]
                if dist < dmin:
                    dmin, pair = dist, (an, bn)
        inside += dmin < thr
        if dmin < best[0]:
            best = (dmin, pair, t[i])
    print(f"\n-- arm-body sphere clearance (meas_q, {len(idx)} samples): min {1000 * best[0]:.1f} mm at {best[2]:.2f} s ({best[1][0]} vs {best[1][1]});"
          f" penalty active (< {1000 * thr:.0f} mm) in {inside} samples")
except Exception as e:  # noqa: BLE001
    print(f"\n(arm-body clearance skipped: {e})")
