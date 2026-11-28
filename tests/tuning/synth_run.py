#!/usr/bin/env python3
"""Synthesize a run_case.sh-style data.npz with a scripted gait sequence, for testing
plot_tracking.py offline (no simulator). Not a physics model: pelvis z/roll are a smooth
gait-locked ripple plus an optional bump at each gait change, joints are sinusoids.

usage: synth_run.py <out_dir> [--sequence walk:8,trot:8,walk:6] [--bump 0.03] [--splice 0.0]
  --sequence  templates and their durations [s] after the 4 s stand (stance template);
              the hold is 20 s in the probe, keep the sum ~<= 20
  --bump      vertical bump amplitude [m] injected at each gait change (0 = none)
  --splice    stance gap [s] inserted at each gait change (what phaseTransitionStanceTime does)
"""
from __future__ import annotations

import argparse
import math
import pathlib

import numpy as np

SETTLE, RAMP, HOLD, STOP, VX = 4.0, 3.0, 20.0, 5.0, 0.5
TEMPL = {  # modeSequence, switchingTimes (config/g1 templates)
    "walk": ([2, 3, 1, 3], [0.0, 0.3, 0.4, 0.7, 0.8]),
    "trot": ([2, 1], [0.0, 0.5, 1.0]),
    "run": ([2, 0, 1, 0], [0.0, 0.25, 0.35, 0.6, 0.7]),
}
JOINTS = [f"{s}_{j}_joint" for s in ("left", "right") for j in
          ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")] + \
         ["waist_yaw_joint"] + [f"{s}_{j}_joint" for s in ("left", "right") for j in
                                ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")]


def mode_schedule(sequence: list[tuple[str, float]], splice: float):
    """-> list of (mode, t_start, t_end) in sim time starting at SETTLE+RAMP."""
    out, t = [], SETTLE + RAMP
    out.append((3, 0.0, t))  # stand
    prev = None
    for name, dur in sequence:
        if prev is not None and splice > 0:
            out.append((3, t, t + splice))
            t += splice
        seq, sw = TEMPL[name]
        end = t + dur
        while t < end - 1e-9:
            for m, a, b in zip(seq, sw[:-1], sw[1:]):
                if t >= end - 1e-9:
                    break
                out.append((m, t, min(t + (b - a), end)))
                t = min(t + (b - a), end)
        prev = name
    out.append((3, t, t + STOP + 2.0))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir")
    ap.add_argument("--sequence", default="walk:8,trot:8,walk:6")
    ap.add_argument("--bump", type=float, default=0.03)
    ap.add_argument("--splice", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    seq = [(s.split(":")[0], float(s.split(":")[1])) for s in args.sequence.split(",")]
    sched = mode_schedule(seq, args.splice)
    t_end = sched[-1][2]
    changes = []
    # gait-change times: start of first phase of each template after the first
    t = SETTLE + RAMP
    for k, (name, dur) in enumerate(seq):
        if k > 0:
            changes.append(t + args.splice)
            t += args.splice
        t += dur

    # tracking stream at 500 Hz
    fs = 500.0
    tt = np.arange(0.0, t_end, 1 / fs)
    mode = np.full(len(tt), 3)
    for m, a, b in sched:
        mode[(tt >= a) & (tt < b)] = m
    valid = (tt >= 1.0).astype(float)
    phase = np.where(tt >= 3.0, 5.0, 2.0)
    policy_t0 = np.floor(tt * 65.0) / 65.0
    names = ["t", "phase", "policy_valid", "plan_mode", "policy_t0"]
    cols = [tt, phase, valid, mode.astype(float), policy_t0]
    swing = np.isin(mode, (1, 2)).astype(float)
    for j in JOINTS:
        q = 0.3 * np.sin(2 * math.pi * 1.25 * tt + hash(j) % 7) * (tt > SETTLE + RAMP)
        n = 0.002 * rng.standard_normal(len(tt))
        tau = 20 * q + 5 * swing
        for grp, sig in (("plan_q", q), ("cmd_q", q), ("meas_q", q + n), ("cmd_qd", np.gradient(q, tt)),
                         ("meas_v", np.gradient(q, tt) + 0.01 * rng.standard_normal(len(tt))),
                         ("plan_tau", tau), ("cmd_tau", tau), ("meas_tau", tau + 0.5 * rng.standard_normal(len(tt)))):
            names.append(f"{grp}/{j}")
            cols.append(sig)
    z = 0.795 + 0.002 * np.sin(2 * math.pi * 2.5 * tt)
    for tc in changes:
        z += args.bump * np.exp(-((tt - tc - 0.3) / 0.12) ** 2)
    vz = np.gradient(z, tt)
    roll = 0.01 * np.sin(2 * math.pi * 1.25 * tt)
    for tc in changes:
        roll += 0.5 * args.bump * np.exp(-((tt - tc - 0.4) / 0.15) ** 2)
    for grp in ("plan_base", "obs_base"):
        for nm, sig in (("z", z), ("vz", vz), ("roll", roll), ("pitch", 0.3 * roll)):
            names.append(f"{grp}/{nm}")
            cols.append(sig)
    for k, f in enumerate(("left_foot", "right_foot")):
        for ax in ("x", "y", "z"):
            for grp in ("plan_foot", "obs_foot"):
                names.append(f"{grp}/{f}/{ax}")
                cols.append(0.02 * np.sin(2 * math.pi * 1.25 * tt))
        names.append(f"plan_wrench/{f}/fz")
        cols.append(np.where(mode == (1 if k else 2), 0.0, 175.0) * (mode != 0))
    track = np.column_stack(cols)

    # ground truth at 100 Hz: t_sim, t_wall, x, y, z, ?, ?, vz, roll, pitch, yaw
    gtt = np.arange(0.0, t_end, 0.01)
    x = np.cumsum(np.interp(gtt, [0, SETTLE, SETTLE + RAMP, SETTLE + RAMP + HOLD, t_end], [0, 0, VX, VX, 0]) * 0.01)
    gt = np.column_stack([gtt, gtt, x, 0 * gtt, np.interp(gtt, tt, z), 0 * gtt, 0 * gtt,
                          np.interp(gtt, tt, vz), np.interp(gtt, tt, roll), 0.3 * np.interp(gtt, tt, roll), 0 * gtt])
    js_t = gtt
    out = pathlib.Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    np.savez(out / "data.npz", phases=np.array([SETTLE, RAMP, HOLD, STOP, VX]), gt=gt,
             track_names=np.array(names), track=track, profile=np.array("vx"),
             js_t=js_t, js_names=np.array(JOINTS), js_q=np.zeros((len(js_t), len(JOINTS))),
             js_v=np.zeros((len(js_t), len(JOINTS))), js_e=np.zeros((len(js_t), len(JOINTS))),
             solver=np.zeros((0, 0)), solver_names=np.array([]))
    print(out / "data.npz", "changes at", [round(c, 2) for c in changes])


if __name__ == "__main__":
    main()
