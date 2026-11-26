#!/usr/bin/env python3
"""One row per heel strike (LF/RF -> STANCE) over whole runs, so the strides that preceded a fall can be
compared with the dozens that did not.

usage: touchdown_table.py <case_dir> [<case_dir> ...] [--min-vx 0.7] [--pre 0.08] [--csv out.csv]

columns (sim-time features around each switch at t_s):
  vx        commanded forward speed at the switch (script knots)
  foot      landing foot side
  lead_ms   how long before the switch the landing foot was already down (last crossing of obs z < 2 mm)
  vz_td     obs vertical speed of the landing foot at that crossing [m/s]
  plan_z    planned foot height at that crossing [cm] (how far the swing plan still wanted to descend)
  ank_err   |meas - cmd| of the landing-foot ankle pitch 20 ms before the switch [rad]
  knee_err  same for the landing-foot knee
  roll_exc  true base roll excursion (max - min) in the --pre window before the switch [deg]
  wroll_tau peak |waist_roll| torque in the --pre window [Nm]
  arm_cmd   mean |cmd shoulder pitch| of both arms at the switch [rad] (template amplitude in play)
  fz20      planned fz of the landing foot 20 ms after the switch [N]
  dpitch40  true base pitch change over the 40 ms after the switch [deg]
  pitchmax  true base pitch max over the 150 ms after the switch [deg]
  FELL      the run's recorded fall lies within 1 s after this switch
The summary at the end gives mean / sd over the non-fall strides and the z-score of each fall stride.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("cases", nargs="+")
ap.add_argument("--min-vx", type=float, default=0.0, help="only strides with commanded vx at least this")
ap.add_argument("--pre", type=float, default=0.08, help="look-back window before the switch [s]")
ap.add_argument("--csv", default=None)
a = ap.parse_args()

COLS = ["case", "t", "vx", "foot", "lead_ms", "vz_td", "plan_z", "ank_err", "knee_err", "roll_exc", "wroll_tau",
        "arm_cmd", "fz20", "dpitch40", "pitchmax", "FELL"]
rows = []


def side(name):
    n = name.lower()
    return "l" if ("left" in n or "_l_" in n or n.endswith("_l") or n.startswith("l_")) else "r"


for case in a.cases:
    npz = os.path.join(case, "data.npz") if os.path.isdir(case) else case
    tag = os.path.basename(case.rstrip("/"))
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
    dt = float(np.median(np.diff(t)))
    sig = lambda n: TR[:, col[n]] if n in col else None
    gt = d["gt"]
    gt = gt[np.argsort(gt[:, 0])]
    gt = gt[np.concatenate([[True], np.diff(gt[:, 0]) > 1e-6])]
    g_pitch = np.interp(t, gt[:, 0], gt[:, 9])
    g_roll = np.interp(t, gt[:, 0], gt[:, 8])
    fell = float(d["fell"]) if "fell" in d.files and np.isfinite(float(d["fell"])) else None
    fell_sim = float(np.interp(fell, gt[:, 1], gt[:, 0])) if fell is not None else None
    K = d["script"] if "script" in d.files and d["script"].size else None
    SETTLE = float(d["phases"][0])
    t_wall = np.interp(t, gt[:, 0], gt[:, 1])
    vx_cmd = np.interp(t_wall - SETTLE, K[:, 0], K[:, 1]) if K is not None else np.full_like(t, np.nan)

    mode = sig("plan_mode")
    feet = sorted({n.split("/")[1] for n in TN if n.startswith("obs_foot/")})
    by_side = {side(f): f for f in feet}
    swinging = {1: "l", 2: "r"}  # RF (1) = right in contact, left swings
    jn = {"l": ("left_ankle_pitch_joint", "left_knee_joint"), "r": ("right_ankle_pitch_joint", "right_knee_joint")}
    sh = [sig("cmd_q/left_shoulder_pitch_joint"), sig("cmd_q/right_shoulder_pitch_joint")]
    wr = sig("meas_tau/waist_roll_joint")
    n_pre = int(round(a.pre / dt))

    sw = np.flatnonzero(np.diff(mode) != 0) + 1
    for k in sw:
        m0, m1 = int(mode[k - 1]), int(mode[k])
        if m0 not in swinging or m1 != 3:
            continue
        if vx_cmd[k] < a.min_vx - 1e-9:
            continue
        s = swinging[m0]
        f = by_side.get(s)
        if f is None:
            continue
        z, vz, pz = sig(f"obs_foot/{f}/z"), sig(f"obs_foot/{f}/vz"), sig(f"plan_foot/{f}/z")
        k0 = max(0, k - int(round(0.2 / dt)))
        down = z[k0:k + 1] < 0.002
        # last crossing into "down" before the switch
        if down[-1]:
            idx = np.flatnonzero(~down)
            kc = k0 + (idx[-1] + 1 if idx.size else 0)
        else:
            kc = k
        lead_ms = 1000 * (t[k] - t[kc])
        k20 = max(0, k - int(round(0.02 / dt)))
        ka, kj = jn[s]
        ank = abs(sig(f"meas_q/{ka}")[k20] - sig(f"cmd_q/{ka}")[k20]) if sig(f"cmd_q/{ka}") is not None else np.nan
        kne = abs(sig(f"meas_q/{kj}")[k20] - sig(f"cmd_q/{kj}")[k20]) if sig(f"cmd_q/{kj}") is not None else np.nan
        kp = max(0, k - n_pre)
        roll_exc = math.degrees(g_roll[kp:k + 1].max() - g_roll[kp:k + 1].min())
        wroll = float(np.abs(wr[kp:k + 1]).max()) if wr is not None else np.nan
        arm = float(np.mean([abs(x[k]) for x in sh if x is not None])) if any(x is not None for x in sh) else np.nan
        fz = sig(f"plan_wrench/{f}/fz")
        k_20 = min(len(t) - 1, k + int(round(0.02 / dt)))
        k_40 = min(len(t) - 1, k + int(round(0.04 / dt)))
        k_150 = min(len(t) - 1, k + int(round(0.15 / dt)))
        rows.append(dict(case=tag, t=t[k], vx=vx_cmd[k], foot=s, lead_ms=lead_ms, vz_td=vz[kc] if vz is not None else np.nan,
                         plan_z=100 * pz[kc] if pz is not None else np.nan, ank_err=ank, knee_err=kne, roll_exc=roll_exc,
                         wroll_tau=wroll, arm_cmd=arm, fz20=fz[k_20] if fz is not None else np.nan,
                         dpitch40=math.degrees(g_pitch[k_40] - g_pitch[k]), pitchmax=math.degrees(g_pitch[k:k_150 + 1].max()),
                         FELL=bool(fell_sim is not None and 0 <= fell_sim - t[k] <= 1.0)))

if not rows:
    sys.exit("no strides matched")

fmt = {"t": "{:7.2f}", "vx": "{:4.1f}", "lead_ms": "{:5.0f}", "vz_td": "{:6.2f}", "plan_z": "{:5.1f}", "ank_err": "{:6.3f}",
       "knee_err": "{:6.3f}", "roll_exc": "{:5.1f}", "wroll_tau": "{:5.0f}", "arm_cmd": "{:5.2f}", "fz20": "{:5.0f}",
       "dpitch40": "{:6.1f}", "pitchmax": "{:6.1f}"}
print(" ".join(f"{c:>{len(fmt.get(c, '{:4}').format(0)) if c in fmt else max(4, len(c))}}" if c != "case" else f"{c:<22}" for c in COLS))
for r in rows:
    out = []
    for c in COLS:
        v = r[c]
        if c == "case":
            out.append(f"{v:<22}")
        elif c in fmt:
            out.append(fmt[c].format(v) if np.isfinite(v) else " " * (len(fmt[c].format(0)) - 1) + "-")
        elif c == "FELL":
            out.append("FELL" if v else "    ")
        else:
            out.append(f"{v:>4}")
    print(" ".join(out))

num = [c for c in COLS if c in fmt and c not in ("t", "vx")]
ok = [r for r in rows if not r["FELL"]]
bad = [r for r in rows if r["FELL"]]
print(f"\n-- summary: {len(ok)} normal strides, {len(bad)} within 1 s of a fall")
print(f"{'feature':<10} {'mean':>8} {'sd':>8}  " + "  ".join(f"{'z ' + b['case'][:12]:>14}" for b in bad))
for c in num:
    vals = np.array([r[c] for r in ok], float)
    vals = vals[np.isfinite(vals)]
    if vals.size < 2:
        continue
    m, sd = vals.mean(), vals.std(ddof=1)
    zs = [((b[c] - m) / sd) if sd > 0 and np.isfinite(b[c]) else float("nan") for b in bad]
    print(f"{c:<10} {m:8.3f} {sd:8.3f}  " + "  ".join(f"{z:+8.1f} ({b[c]:6.2f})" for z, b in zip(zs, bad)))

if a.csv:
    import csv
    with open(a.csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLS)
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {a.csv}")
