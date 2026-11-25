#!/usr/bin/env python3
"""Text diagnostics of a run_case.sh case for the whole-body feedback layer (Phase 2.B).

usage: wbc_diag.py <case_dir> [<case_dir> ...]

Prints, per case: the layer's health (wbc/* columns), where the applied-torque HF sits per joint
(cmd vs residual PD vs total), base plan-vs-observed tracking in the walk window, the command
events (plan pitch / z vs observed), and - when the run fell - the last second before the fall.
Reads <case>/data.npz (bag_to_npz.py) and metrics.json; no ROS needed.
"""
import json
import math
import pathlib
import sys

import numpy as np


def band(tt, x, lo=4.0, hi=40.0, fs=500.0):
    if len(tt) < 100:
        return float("nan")
    tu = np.arange(tt[0], tt[-1], 1.0 / fs)
    xu = np.interp(tu, tt, x)
    xu = xu - np.polyval(np.polyfit(tu - tu[0], xu, 1), tu - tu[0])
    f = np.fft.rfftfreq(len(xu), 1.0 / fs)
    X = np.fft.rfft(xu)
    X[(f < lo) | (f > hi)] = 0
    return float(np.sqrt(np.mean(np.fft.irfft(X, len(xu)) ** 2)))


def diag(case: pathlib.Path):
    print(f"\n######## {case}")
    data = case / "data.npz"
    if not data.is_file():
        print("  no data.npz")
        return
    d = np.load(data, allow_pickle=False)
    met = json.loads((case / "metrics.json").read_text()) if (case / "metrics.json").is_file() else {}
    prof = str(d["profile"]) if "profile" in d.files else "vx"
    settle, ramp, hold, stop, vx = d["phases"]
    gt = d["gt"]
    w2s = lambda w: float(np.interp(w, gt[:, 1], gt[:, 0]))
    fell_wall = float(d["fell"]) if "fell" in d.files else float("nan")
    fell_sim = w2s(fell_wall) if not np.isnan(fell_wall) else float("nan")
    print(f"  profile {prof} vx {vx} | survived {met.get('survived')} fell_at {met.get('fell_at')} (sim {fell_sim:.2f})")
    names = list(d["track_names"])
    T = d["track"]
    if T.size == 0:
        print("  no tracking rows")
        return
    col = {n: i for i, n in enumerate(names)}
    c = lambda n: T[:, col[n]]
    t = c("t")
    valid = c("policy_valid") > 0
    w0, w1 = w2s(settle + ramp + 3.0), w2s(settle + ramp + hold)
    walk = (t >= w0) & (t <= w1) & valid
    if not np.isnan(fell_sim):
        walk &= t < fell_sim - 0.5
    loco = valid.copy()
    if walk.sum() < 200:
        print(f"  walk window too short ({walk.sum()} ticks); using all policy-valid ticks")
        walk = valid

    # 1. layer health
    if "wbc/active" in col:
        act = c("wbc/active")[loco]
        res = c("wbc/residual")[loco]
        clips = c("wbc/clips")[loco]
        alpha = c("wbc/gain_alpha")
        res = res[np.isfinite(res)]
        clips = clips[np.isfinite(clips)]
        t_full = t[alpha >= 0.999]
        print(f"  wbc: active {100 * np.mean(act > 0):.2f} % of policy ticks | residual median {np.median(res):.2e} "
              f"p99 {np.percentile(res, 99):.2e} max {np.max(res):.2e} | clips mean {np.mean(clips):.2f}, ticks with clips "
              f"{100 * np.mean(clips > 0):.1f} % | gains fully residual from t={t_full[0]:.2f} s" if len(t_full) else
              f"  wbc: active {100 * np.mean(act > 0):.2f} % | gain ramp never completed (alpha max {np.max(alpha):.2f})")
    else:
        print("  wbc columns absent (layer disabled in this case)")

    # 2. torque HF per joint
    joints = [n.split("/", 1)[1] for n in names if n.startswith("meas_tau/")]
    tw = t[walk]
    rows = []
    for j in joints:
        meas = c("meas_tau/" + j)[walk]
        cmd = c("cmd_tau/" + j)[walk]
        rows.append((j.replace("_joint", ""), band(tw, meas), band(tw, cmd), band(tw, meas - cmd), np.max(np.abs(meas))))
    rows.sort(key=lambda r: -r[1])
    print("  torque HF 4-40 Hz [Nm] (walk window), top 10 by total:   joint  total  cmd  pd  |max|")
    for r in rows[:10]:
        print(f"    {r[0]:22s} {r[1]:6.2f} {r[2]:6.2f} {r[3]:6.2f} {r[4]:7.1f}")
    legs = [r for r in rows if any(k in r[0] for k in ("hip", "knee", "ankle"))]
    arms = [r for r in rows if any(k in r[0] for k in ("shoulder", "elbow"))]
    for gname, g in (("legs", legs), ("arms", arms)):
        if g:
            print(f"  {gname}: mean total {np.mean([r[1] for r in g]):.3f} cmd {np.mean([r[2] for r in g]):.3f} pd {np.mean([r[3] for r in g]):.3f}")

    # 3. base tracking
    def err(axis):
        return c("obs_base/" + axis)[walk] - c("plan_base/" + axis)[walk]
    for axis, scale, unit in (("z", 1000, "mm"), ("pitch", 180 / math.pi, "deg"), ("roll", 180 / math.pi, "deg")):
        e = err(axis) * scale
        plan = c("plan_base/" + axis)[walk] * scale
        print(f"  base {axis:5s}: obs-plan rms {np.sqrt(np.mean(e ** 2)):.3f} max {np.max(np.abs(e)):.3f} {unit} | plan p-p {np.ptp(plan):.3f} {unit}")
    for f in [n.split("/")[1] for n in names if n.startswith("plan_foot/") and n.endswith("/z")]:
        e = (c(f"obs_foot/{f}/z") - c(f"plan_foot/{f}/z"))[walk] * 1000
        print(f"  foot {f}: z obs-plan rms {np.sqrt(np.mean(e ** 2)):.2f} max {np.max(np.abs(e)):.2f} mm")

    # 4. command events
    events = []
    if prof == "stops":
        tw_ = settle + ramp
        while tw_ + 4.0 < settle + ramp + hold:
            events.append(("step_down", tw_ + 4.0))
            if tw_ + 7.0 < settle + ramp + hold:
                events.append(("step_up", tw_ + 7.0))
            tw_ += 7.0
    else:
        events = [("ramp_up", settle), ("ramp_down", settle + ramp + hold)]
    print("  events [-0.3, +2.0] s: obs pitch max | plan pitch max | obs-plan pitch max | obs z p-p | plan z p-p | ankle |tau| max")
    for kind, tw_ in events:
        ts = w2s(tw_)
        if not np.isnan(fell_sim) and fell_sim < ts:
            print(f"    {kind:10s} t={ts:6.2f}: after the fall")
            continue
        m = (t >= ts - 0.3) & (t <= ts + 2.0) & valid
        if m.sum() < 50:
            print(f"    {kind:10s} t={ts:6.2f}: no data")
            continue
        op, pp = c("obs_base/pitch")[m], c("plan_base/pitch")[m]
        oz, pz = c("obs_base/z")[m], c("plan_base/z")[m]
        ank = max(np.max(np.abs(c("meas_tau/" + j)[m])) for j in joints if "ankle" in j) if any("ankle" in j for j in joints) else float("nan")
        ref = np.median(c("obs_base/pitch")[walk])
        print(f"    {kind:10s} t={ts:6.2f}: {math.degrees(np.max(np.abs(op - ref))):6.2f} | {math.degrees(np.max(np.abs(pp - ref))):6.2f} | "
              f"{math.degrees(np.max(np.abs(op - pp))):6.2f} deg | {1000 * np.ptp(oz):6.1f} | {1000 * np.ptp(pz):6.1f} mm | {ank:5.1f} Nm")

    # 5. the last second before a fall
    if not np.isnan(fell_sim):
        m = (t >= fell_sim - 1.0) & (t <= fell_sim) & valid
        if m.sum() > 10:
            print(f"  last 1.0 s before the fall (t {fell_sim - 1.0:.2f}..{fell_sim:.2f}):")
            for axis, scale, unit in (("pitch", 180 / math.pi, "deg"), ("roll", 180 / math.pi, "deg"), ("z", 1000, "mm")):
                o, p = c("obs_base/" + axis)[m] * scale, c("plan_base/" + axis)[m] * scale
                k = np.linspace(0, m.sum() - 1, 6).astype(int)
                print(f"    {axis:5s} obs  " + " ".join(f"{v:7.2f}" for v in o[k]) + f"  {unit}")
                print(f"    {axis:5s} plan " + " ".join(f"{v:7.2f}" for v in p[k]))
            if "plan_mode" in col:
                print(f"    plan_mode sequence: {np.unique(c('plan_mode')[m]).astype(int).tolist()}")
            worst = sorted(((np.max(np.abs(c('meas_tau/' + j)[m])), j) for j in joints), reverse=True)[:4]
            print("    |meas_tau| max: " + ", ".join(f"{j.replace('_joint', '')} {v:.1f}" for v, j in worst))
            worst = sorted(((math.degrees(np.max(np.abs(c('meas_q/' + j)[m] - c('cmd_q/' + j)[m]))), j) for j in joints), reverse=True)[:4]
            print("    |q err| max: " + ", ".join(f"{j.replace('_joint', '')} {v:.1f} deg" for v, j in worst))
            if "wbc/residual" in col:
                r = c("wbc/residual")[m]
                print(f"    wbc residual max {np.nanmax(r):.2e}, clips max {np.nanmax(c('wbc/clips')[m]):.0f}, "
                      f"active {100 * np.mean(c('wbc/active')[m] > 0):.0f} %")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for arg in sys.argv[1:]:
        diag(pathlib.Path(arg))
