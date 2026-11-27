#!/usr/bin/env python3
"""Compare command-space suite results of candidate tags against a baseline tag, per script and segment.

usage: compare_suite.py <results_root> <baseline_tag> <candidate_tag> [more candidates] [--scripts a b ...]
                        [--primary KEY:RATIO ...] [--guard KEY:max|min:RATIO:SLACK ...]

Reads <root>/<tag>__<script>/suite.json. Aggregates over the whole suite (all scripts):
  steady_jerk_mean, steady_acc_hf_mean, steady_tau_rate_mean      (means over steady segments)
  transition_jerk_max, transition_acc_hf_max, transition_tau_rate_max (max over transitions)
  transition_jerk_mean, transition_acc_hf_mean
  v_err_max (worst command tracking of any segment), falls
  solve_hz (min over scripts of the MPC_TIMING actual rate in <case>/summary.txt; nan if absent)
  hl_leg, knee_ms, hip_rom, arm_amp (human-likeness of the forward-walking holds, see score_suite.py; informational)
  steady_leg_acc_mean, steady_leg_jerk_mean, steady_arm_acc_mean, steady_arm_pp_mean (limb motion, see
  score_suite.py; printed, and the limbs guards below keep a candidate from winning by moving the
  abruptness from the pelvis into the limbs)
Default gates -- a candidate WINS when every guard holds and at least one primary improves by its
ratio while no primary gets worse than 1.10x:
  primaries: transition_jerk_max 0.8, transition_acc_hf_max 0.8, steady_jerk_mean 0.85, steady_acc_hf_mean 0.85
  guards:    falls == 0, v_err_max <= 1.2x + 0.03, steady_tau_rate_mean <= 1.15x, transition_tau_rate_max <= 1.15x,
             solve_hz >= 0.9x (formulation changes must not starve the solver),
             steady_leg_jerk_mean <= 1.15x, steady_arm_acc_mean <= 1.15x (limbs; skipped when the baseline lacks them)
The per-segment table prints every segment of every script side by side (jerk | acc_hf | tau_rate).
"""
import argparse
import json
import math
import re
from pathlib import Path

import numpy as np

AGG_KEYS = ["steady_jerk_mean", "steady_acc_hf_mean", "steady_tau_rate_mean", "transition_jerk_max",
            "transition_acc_hf_max", "transition_tau_rate_max", "transition_jerk_mean", "transition_acc_hf_mean", "v_err_max", "falls", "solve_hz",
            "steady_leg_acc_mean", "steady_leg_jerk_mean", "steady_arm_acc_mean", "steady_arm_pp_mean",
            "hl_leg", "knee_ms", "hip_rom", "arm_amp"]


def load(root: Path, tag: str, scripts):
    out = {}
    for s in scripts:
        f = root / f"{tag}__{s}" / "suite.json"
        if f.is_file():
            out[s] = json.loads(f.read_text())
            summary = f.parent / "summary.txt"
            m = re.search(r"MPC_TIMING\].*?actual=([0-9.]+) Hz", summary.read_text()) if summary.is_file() else None
            out[s]["solve_hz"] = float(m.group(1)) if m else math.nan
    return out


def aggregate(runs):
    steady = [r for j in runs.values() for r in j["segments"] if r["kind"] == "steady"]
    trans = [r for j in runs.values() for r in j["segments"] if r["kind"] == "transition"]
    vals = lambda rows, k: [r[k] for r in rows if isinstance(r.get(k), (int, float)) and math.isfinite(r[k])]
    agg = {
        "steady_jerk_mean": np.mean(vals(steady, "jerk")) if steady else math.nan,
        "steady_acc_hf_mean": np.mean(vals(steady, "acc_hf")) if steady else math.nan,
        "steady_tau_rate_mean": np.mean(vals(steady, "tau_rate")) if vals(steady, "tau_rate") else math.nan,
        "transition_jerk_max": max(vals(trans, "jerk"), default=math.nan),
        "transition_acc_hf_max": max(vals(trans, "acc_hf"), default=math.nan),
        "transition_tau_rate_max": max(vals(trans, "tau_rate"), default=math.nan),
        "transition_jerk_mean": np.mean(vals(trans, "jerk")) if trans else math.nan,
        "transition_acc_hf_mean": np.mean(vals(trans, "acc_hf")) if trans else math.nan,
        "v_err_max": max(vals(steady + trans, "v_err"), default=math.nan),
        "falls": sum(0 if j["agg"]["survived"] else 1 for j in runs.values()),
        "solve_hz": min([j["solve_hz"] for j in runs.values() if math.isfinite(j.get("solve_hz", math.nan))], default=math.nan),
        "steady_leg_acc_mean": np.mean(vals(steady, "leg_acc")) if vals(steady, "leg_acc") else math.nan,
        "steady_leg_jerk_mean": np.mean(vals(steady, "leg_jerk")) if vals(steady, "leg_jerk") else math.nan,
        "steady_arm_acc_mean": np.mean(vals(steady, "arm_acc")) if vals(steady, "arm_acc") else math.nan,
        "hl_leg": np.mean(vals(steady, "hl_leg")) if vals(steady, "hl_leg") else math.nan,
        "knee_ms": np.mean(vals(steady, "knee_ms")) if vals(steady, "knee_ms") else math.nan,
        "hip_rom": np.mean(vals(steady, "hip_rom")) if vals(steady, "hip_rom") else math.nan,
        "arm_amp": np.mean(vals(steady, "arm_amp")) if vals(steady, "arm_amp") else math.nan,
        "steady_arm_pp_mean": np.mean(vals(steady, "arm_pp")) if vals(steady, "arm_pp") else math.nan,
        "scripts": len(runs),
    }
    return {k: (float(v) if isinstance(v, (np.floating, float)) else v) for k, v in agg.items()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", type=Path); ap.add_argument("base"); ap.add_argument("cands", nargs="+")
    ap.add_argument("--scripts", nargs="+", default=["speeds", "turns", "lateral", "stops", "omni"])
    ap.add_argument("--primary", action="append", default=None)
    ap.add_argument("--guard", action="append", default=None)
    a = ap.parse_args()
    cands = [c for c in a.cands if not c.startswith("--")]
    primaries = [(p.split(":")[0], float(p.split(":")[1])) for p in (a.primary or
                 ["transition_jerk_max:0.8", "transition_acc_hf_max:0.8", "steady_jerk_mean:0.85", "steady_acc_hf_mean:0.85"])]
    guards = []
    for g in (a.guard or ["v_err_max:max:1.2:0.03", "steady_tau_rate_mean:max:1.15:0", "transition_tau_rate_max:max:1.15:0",
                          "solve_hz:min:0.9:0", "steady_leg_jerk_mean:max:1.15:0", "steady_arm_acc_mean:max:1.15:0"]):
        k, kind, ratio, slack = g.split(":"); guards.append((k, kind, float(ratio), float(slack)))

    base_runs = load(a.root, a.base, a.scripts)
    if not base_runs:
        raise SystemExit(f"no suite.json for baseline {a.base} under {a.root}")
    B = aggregate(base_runs)
    print(f"baseline {a.base}: " + " | ".join(f"{k} {B[k]:.3f}" if isinstance(B[k], float) else f"{k} {B[k]}" for k in AGG_KEYS))
    print(f"{'candidate':14s} " + " ".join(f"{k[:12]:>12s}" for k in AGG_KEYS) + "  verdict")
    verdicts = {}
    for c in cands:
        R = aggregate(load(a.root, c, a.scripts))
        fails, improved = [], []
        if R["falls"] > 0:
            fails.append(f"falls {R['falls']}")
        for k, kind, ratio, slack in guards:
            if not (isinstance(R[k], float) and math.isfinite(R[k]) and math.isfinite(B[k])):
                continue
            lim = B[k] * ratio + slack
            if (kind == "max" and R[k] > lim) or (kind == "min" and R[k] < lim):
                fails.append(f"guard {k}: {B[k]:.3f} -> {R[k]:.3f} (limit {kind} {lim:.3f})")
        for k, ratio in primaries:
            if not (isinstance(R[k], float) and math.isfinite(R[k]) and math.isfinite(B[k])):
                continue
            if R[k] <= B[k] * ratio:
                improved.append(k)
            elif R[k] > B[k] * 1.10:
                fails.append(f"primary {k}: {B[k]:.3f} -> {R[k]:.3f} (worse than 1.10x)")
        verdict = ("WINNER (" + ", ".join(improved) + ")") if improved and not fails else ("guards-fail" if fails else "no gain")
        verdicts[c] = (verdict, fails)
        print(f"{c:14s} " + " ".join(f"{R[k]:12.3f}" if isinstance(R[k], float) else f"{R[k]:12d}" for k in AGG_KEYS) + f"  {verdict}")
    for c, (v, fails) in verdicts.items():
        for f in fails:
            print(f"  [FAIL] {c}: {f}")
    # per-segment table
    def longest(runs, tags, s):
        return max((runs[t].get(s, {}).get("segments", []) for t in tags), key=len)

    print("\nper segment (jerk | acc_hf | tau_rate; F = fell in this segment):")
    tags = [a.base] + cands
    runs = {t: load(a.root, t, a.scripts) for t in tags}
    for s in a.scripts:
        if s not in base_runs:
            continue
        print(f"-- {s}")
        # rows from the longest run: a baseline that fell early must not hide the candidates' later segments
        for i, seg in enumerate(longest(runs, tags, s)):
            cells = []
            for t in tags:
                segs = runs[t].get(s, {}).get("segments", [])
                r = segs[i] if i < len(segs) else None
                cells.append(f"{r['jerk']:6.2f} {r['acc_hf']:6.3f} {r.get('tau_rate', float('nan')):7.0f}" + ("F" if r.get("fell") else " ") if r else " " * 22)
            print(f"  {seg['label']:34s} {seg['kind'][:5]:5s} | " + " | ".join(cells))
    # limbs table (only when the baseline has joint-state metrics)
    if any("leg_acc" in seg for j in base_runs.values() for seg in j["segments"]):
        print("\nper segment limbs (z_m m | step_l m | v_err | leg_acc | leg_jerk | arm_acc | arm_pp deg || hl_leg | knee_ms | hip_rom | knee_sw | arm_amp deg):")
        for s in a.scripts:
            if s not in base_runs:
                continue
            print(f"-- {s}")
            for i, seg in enumerate(longest(runs, tags, s)):
                cells = []
                for t in tags:
                    segs = runs[t].get(s, {}).get("segments", [])
                    r = segs[i] if i < len(segs) else None
                    nan = float("nan")
                    cells.append(f"{r.get('z_m', nan):5.3f} {r.get('step_l', nan):5.2f} {r.get('v_err', nan):5.2f} {r.get('leg_acc', nan):6.2f} {r.get('leg_jerk', nan):6.0f} {r.get('arm_acc', nan):6.2f} {r.get('arm_pp', nan):5.1f} || "
                                 f"{r.get('hl_leg', nan):5.1f} {r.get('knee_ms', nan):5.1f} {r.get('hip_rom', nan):5.1f} {r.get('knee_sw', nan):5.1f} {r.get('arm_amp', nan):5.1f}" if r else " " * 71)
                print(f"  {seg['label']:34s} {seg['kind'][:5]:5s} | " + " | ".join(cells))
    print("\nwinners: " + (", ".join(c for c, (v, _) in verdicts.items() if v.startswith("WINNER")) or "NONE"))


if __name__ == "__main__":
    main()
