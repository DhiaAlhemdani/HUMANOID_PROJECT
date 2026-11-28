#!/usr/bin/env python3
"""Event-aligned averages around the planned contact switches: what the plan's foot forces, the
measured foot and the pelvis do in the +-150 ms around every lift-off and every touchdown switch.

usage: switch_profile.py <case_dir> [<case_dir> ...] [--min-vx 0.4] [--speeds 0.2,0.5,0.8] [--step 0.01]

Rows are time relative to the planned switch (negative = before). Columns:
  fz_sw     planned vertical force of the switching foot [N] (the one lifting off / landing)
  fz_ot     planned vertical force of the other foot [N]
  z_sw      measured height of the switching foot [mm] (obs_foot)
  plan_z    planned height of the switching foot [mm]
  jerk      pelvis 0-15 Hz jerk magnitude [m/s^3], ground truth
  az        pelvis vertical acceleration [m/s^2], ground truth (4-40 Hz band)
  alpha     wbc/contact_alpha (1 = layer torque, 0 = plan torque)
Averages over all events of the class inside the holds at the listed speeds (one table per class and
speed), so a force step at the switch or a foot that is down long before it shows as a shape, not a
number.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("cases", nargs="+")
ap.add_argument("--min-vx", type=float, default=0.4)
ap.add_argument("--speeds", default="0.2,0.5,0.8")
ap.add_argument("--step", type=float, default=0.01)
ap.add_argument("--span", type=float, default=0.15)
a = ap.parse_args()
FS = 500.0
SPEEDS = [float(x) for x in a.speeds.split(",")]
REL = np.arange(-a.span, a.span + 1e-9, a.step)


def band_series(tt, x, lo=0.0, hi=15.0):
    tu = np.arange(tt[0], tt[-1], 1.0 / FS)
    xu = np.interp(tu, tt, x)
    xu = xu - np.polyval(np.polyfit(tu - tu[0], xu, 1), tu - tu[0])
    f = np.fft.rfftfreq(len(xu), 1.0 / FS)
    X = np.fft.rfft(xu)
    X[(f < lo) | (f > hi)] = 0
    return tu, np.fft.irfft(X, len(xu))


def side(name):
    n = name.lower()
    return "l" if ("left" in n or "_l_" in n or n.endswith("_l") or n.startswith("l_")) else "r"


COLS = ["fz_sw", "fz_ot", "z_sw", "plan_z", "jerk", "az", "alpha"]
acc = {}  # (case, class, speed) -> list of arrays [len(REL), len(COLS)]
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
    sig = lambda n: TR[:, col[n]] if n in col else None
    gt = d["gt"]
    gt = gt[np.argsort(gt[:, 0])]
    gt = gt[np.concatenate([[True], np.diff(gt[:, 0]) > 1e-6])]
    SETTLE = float(d["phases"][0])
    w2s = lambda w: float(np.interp(w, gt[:, 1], gt[:, 0]))
    K = d["script"] if "script" in d.files and d["script"].size else None
    if K is None:
        print(f"{tag}: no script in data.npz", file=sys.stderr)
        continue
    mode = sig("plan_mode")
    if mode is None:
        print(f"{tag}: no plan_mode", file=sys.stderr)
        continue
    mode = np.where(np.isfinite(mode), mode, -1.0)
    feet = sorted({n.split("/")[1] for n in TN if n.startswith("obs_foot/")})
    by_side = {side(f): f for f in feet}
    swinging = {1: "l", 2: "r"}
    alpha = sig("wbc/contact_alpha")

    for p, q in zip(K[:-1], K[1:]):
        if not (np.allclose(p[1:], q[1:]) and q[0] - p[0] >= 2.5 and abs(p[1]) >= a.min_vx):
            continue
        speed = min(SPEEDS, key=lambda s: abs(s - abs(p[1])))
        if abs(speed - abs(p[1])) > 0.05:
            continue
        s0, s1 = w2s(SETTLE + p[0] + 1.5), w2s(SETTLE + q[0])
        g = gt[(gt[:, 0] >= s0 - 0.5) & (gt[:, 0] <= s1 + 0.5)]
        if len(g) < 100:
            continue
        tt = g[:, 0]
        v = np.column_stack([np.gradient(g[:, k], tt) for k in (2, 3, 4)])
        ac = np.column_stack([np.gradient(v[:, k], tt) for k in range(3)])
        jr = np.column_stack([np.gradient(ac[:, k], tt) for k in range(3)])
        tu, jx = band_series(tt, jr[:, 0]); _, jy = band_series(tt, jr[:, 1]); _, jz = band_series(tt, jr[:, 2])
        jmag = np.sqrt(jx ** 2 + jy ** 2 + jz ** 2)
        _, azb = band_series(tt, ac[:, 2], 4, 40)
        idx = np.flatnonzero((t >= s0) & (t <= s1))
        for k in idx[1:]:
            m0, m1 = int(mode[k - 1]), int(mode[k])
            if m0 == m1:
                continue
            if m0 in swinging and m1 == 3:
                cls, sw = "touchdown", swinging[m0]
            elif m0 == 3 and m1 in swinging:
                cls, sw = "liftoff", swinging[m1]
            else:
                continue
            f_sw = by_side.get(sw)
            f_ot = by_side.get("r" if sw == "l" else "l")
            if f_sw is None or f_ot is None:
                continue
            ts = t[k]
            tq = ts + REL
            row = np.full((len(REL), len(COLS)), np.nan)
            def at(sg, src_t):
                return np.interp(tq, src_t, sg) if sg is not None else np.full(len(REL), np.nan)
            row[:, 0] = at(sig(f"plan_wrench/{f_sw}/fz"), t)
            row[:, 1] = at(sig(f"plan_wrench/{f_ot}/fz"), t)
            row[:, 2] = 1000 * at(sig(f"obs_foot/{f_sw}/z"), t)
            row[:, 3] = 1000 * at(sig(f"plan_foot/{f_sw}/z"), t)
            row[:, 4] = at(jmag, tu)
            row[:, 5] = at(azb, tu)
            row[:, 6] = at(alpha, t)
            acc.setdefault((tag, cls, speed), []).append(row)

import warnings
warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN columns (no wbc stream) are printed as nan
for (tag, cls, speed), rows in sorted(acc.items(), key=lambda kv: (kv[0][0], kv[0][2], kv[0][1])):
    M = np.nanmean(np.stack(rows), axis=0)
    after = M[REL > 0.05, 0]
    if cls == "liftoff" and np.isfinite(after).any() and np.nanmean(after) > 20:
        print(f"\n!! {tag}: the planned force of the 'lifting' foot stays {np.nanmean(after):.0f} N after lift-off - "
              f"the side convention (plan_mode 1 = RF = right in contact) does not match this recording")
    print(f"\n== {tag} | {cls} | hold {speed:.1f} m/s | {len(rows)} events (mean)")
    print(f"{'t_ms':>6s} " + " ".join(f"{c:>7s}" for c in COLS))
    for r, m in zip(REL, M):
        print(f"{1000 * r:6.0f} " + " ".join(f"{x:7.1f}" if abs(x) >= 10 or np.isnan(x) else f"{x:7.2f}" for x in m))
