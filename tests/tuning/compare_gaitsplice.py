#!/usr/bin/env python3
"""Score the Phase 1.5 gait-transition sweep (gait.phaseTransitionStanceTime) from
run_gaitsplice_sweep.sh.

usage: compare_gaitsplice.py <baseline_dir> <candidate_dir> [candidate_dir ...]

Each dir is a run_case.sh result dir (metrics.json + diag.json with the `gait` section from
plot_tracking.py). The lever only acts at locomotion<->locomotion gait changes (walk<->trot ...),
so the primaries are the transition-window metrics (worst change in the run, [-0.5, +1.5] s
around the first step of the new template), ANY-OF:

  PRIMARIES  gait.transition_acc_hf_max   <= 0.85x   (bounce at the switch)
             gait.transition_z_pp_mm_max  <= 0.85x   (height excursion at the switch)
             gait.transition_roll_max_deg <= 0.85x   (lateral lurch at the switch)
  GUARDS     survived; gait.changes_locomotion == baseline (same transitions exercised — else
             the run is not comparable and is reported as such); non-improved primaries not worse
             (<= 1.10x / 1.10x+0.5 mm / 1.10x+0.05 deg); gait.first_step_asym_max <= 1.20x + 0.05;
             steady-state guards: walk.pelvis_acc_hf <= 1.10x, roll <= 1.15x+0.05, jump@switch <=
             1.15x+0.02, leg PD HF <= 1.10x+0.1, track err <= 1.20x+0.05, plan z sd / z-track <=
             1.10x+0.5, arm swing <= 1.20x+1.0, vx >= 0.90x, solve >= 0.90x.

Exit 0 iff at least one candidate wins. Stdlib only; load() never raises.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

PRIMARIES = [  # (path, improve ratio, not-worse ratio, not-worse slack)
    ("gait.transition_acc_hf_max", 0.85, 1.10, 0.0),
    ("gait.transition_z_pp_mm_max", 0.85, 1.10, 0.5),
    ("gait.transition_roll_max_deg", 0.85, 1.10, 0.05),
]
GUARDS = [  # (path, kind, ratio, slack)
    ("gait.first_step_asym_max", "max", 1.20, 0.05),
    ("walk.pelvis_acc_hf", "max", 1.10, 0.0),
    ("walk.roll_rms_deg", "max", 1.15, 0.05),
    ("legs.cmd_tau_jump_at_switch", "max", 1.15, 0.02),
    ("legs.pd_tau_hf", "max", 1.10, 0.1),
    ("legs.track_err_q_deg", "max", 1.20, 0.05),
    ("base.plan_z_sd_mm", "max", 1.10, 0.5),
    ("base.z_track_err_mm", "max", 1.10, 0.5),
    ("walk.arm_swing_pp_deg", "max", 1.20, 1.0),
    ("walk.vx_mean", "min", 0.90, 0.0),
    ("solver.solve_hz", "min", 0.90, 0.0),
]


def load(d: Path) -> tuple[dict | None, str | None]:
    if not d.is_dir():
        return None, f"missing directory {d}"
    missing = [f for f in ("metrics.json", "diag.json") if not (d / f).is_file()]
    if missing:
        return None, f"missing {', '.join(missing)} (run ended before scoring; see {d}/summary.txt)"
    try:
        met = json.loads((d / "metrics.json").read_text())
        diag = json.loads((d / "diag.json").read_text())
    except (json.JSONDecodeError, OSError) as e:
        return None, f"unreadable metrics: {e}"
    if "gait" not in diag:
        return None, "diag.json has no `gait` section — re-score with the current plot_tracking.py"
    flat: dict = {"survived": met.get("survived"), "fell_at": met.get("fell_at")}
    for k, v in (met.get("walk") or {}).items():
        flat[f"walk.{k}"] = v
    for section in ("base", "legs", "arms", "solver", "gait"):
        for k, v in (diag.get(section) or {}).items():
            if not isinstance(v, (dict, list)):
                flat[f"{section}.{k}"] = v
    flat["gait.sequence"] = (diag["gait"] or {}).get("sequence")
    return flat, None


def score(base: dict, cand: dict):
    rows = [("survived", bool(cand.get("survived")), f"fell_at={cand.get('fell_at')}")]
    nb, nc = base.get("gait.changes_locomotion"), cand.get("gait.changes_locomotion")
    comparable = nb is not None and nb == nc and nb > 0
    rows.append(("guard gait.changes_locomotion", comparable,
                 f"base {nb} ({base.get('gait.sequence')}) vs cand {nc} ({cand.get('gait.sequence')})"
                 + ("" if nb else " — baseline has no gait-to-gait transition; raise vx so walk->trot happens")))
    improved: list[str] = []
    guards_ok = comparable
    for path, imp, nw_ratio, nw_slack in PRIMARIES:
        b, c = base.get(path), cand.get(path)
        if b is None or c is None:
            rows.append((f"primary {path}", False, f"missing (base={b}, cand={c})"))
            guards_ok = False
            continue
        if c <= imp * b:
            improved.append(path)
            rows.append((f"primary {path}", True, f"{b} -> {c} (improve limit {imp}x)"))
        else:
            ok = c <= nw_ratio * b + nw_slack
            guards_ok &= ok
            rows.append((f"primary {path}", ok, f"{b} -> {c} (not improved; not-worse limit {nw_ratio}x + {nw_slack})"))
    for path, kind, ratio, slack in GUARDS:
        b, c = base.get(path), cand.get(path)
        if b is None or c is None:
            rows.append((f"guard {path}", False, f"missing (base={b}, cand={c})"))
            guards_ok = False
            continue
        ok = (c <= ratio * b + slack) if kind == "max" else (c >= ratio * b)
        guards_ok &= ok
        rows.append((f"guard {path}", ok, f"{b} -> {c} (limit {'<= ' + str(ratio) + 'x + ' + str(slack) if kind == 'max' else '>= ' + str(ratio) + 'x'})"))
    wins = bool(cand.get("survived")) and guards_ok and bool(improved)
    return wins, improved, rows


def num(d: dict, path: str) -> float:
    v = d.get(path)
    return float(v) if isinstance(v, (int, float)) else float("nan")


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__)
        return 2
    base, err = load(Path(argv[1]))
    if base is None:
        print(f"BASELINE FAILED: {argv[1]}: {err}")
        return 2
    print(f"baseline: {argv[1]}")
    print(f"  gait {base.get('gait.sequence')} transitions {base.get('gait.changes_locomotion')} | "
          f"transition acc_hf {base.get('gait.transition_acc_hf_max')} z_pp {base.get('gait.transition_z_pp_mm_max')} mm "
          f"roll {base.get('gait.transition_roll_max_deg')} deg asym {base.get('gait.first_step_asym_max')} | "
          f"steady acc_hf {base.get('walk.pelvis_acc_hf')} roll {base.get('walk.roll_rms_deg')} | "
          f"vx {base.get('walk.vx_mean')} | solve {base.get('solver.solve_hz')} Hz\n")
    hdr = (f"{'candidate':12s} {'n':>2s} {'t_acc':>6s} {'t_zpp':>6s} {'t_roll':>6s} {'asym':>6s} "
           f"{'acc_hf':>7s} {'roll':>6s} {'jump':>6s} {'vx':>6s}  verdict")
    print(hdr)
    print("-" * len(hdr))
    winners, detail = [], []
    cands = [(p, *load(Path(p))) for p in argv[2:]]
    for path, cand, cerr in cands:
        name = Path(path).name
        if cand is None:
            detail.append((name, [("load", False, cerr or "unknown error")]))
            print(f"{name:12s} {'-':>2s} {'-':>6s} {'-':>6s} {'-':>6s} {'-':>6s} {'-':>7s} {'-':>6s} {'-':>6s} {'-':>6s}  ERROR")
            continue
        wins, improved, rows = score(base, cand)
        detail.append((name, rows))
        if wins:
            winners.append(name)
        verdict = ("WINNER (" + ", ".join(p.split(".")[-1].replace("transition_", "") for p in improved) + ")") if wins \
            else ("not comparable" if not rows[1][1] else ("guards-fail" if not all(r[1] for r in rows) else "no gain"))
        print(f"{name:12s} {int(num(cand, 'gait.changes_locomotion')) if cand.get('gait.changes_locomotion') is not None else 0:2d} "
              f"{num(cand, 'gait.transition_acc_hf_max'):6.3f} {num(cand, 'gait.transition_z_pp_mm_max'):6.1f} "
              f"{num(cand, 'gait.transition_roll_max_deg'):6.2f} {num(cand, 'gait.first_step_asym_max'):6.3f} "
              f"{num(cand, 'walk.pelvis_acc_hf'):7.3f} {num(cand, 'walk.roll_rms_deg'):6.2f} "
              f"{num(cand, 'legs.cmd_tau_jump_at_switch'):6.3f} {num(cand, 'walk.vx_mean'):6.3f}  {verdict}")
    print()
    for name, rows in detail:
        for gname, ok, gdet in rows:
            if not ok:
                print(f"  [FAIL] {name}: {gname}: {gdet}")
    print(f"\nwinners: {', '.join(winners) if winners else 'NONE'} "
          f"({len(winners)}/{len(cands)} candidates improve a transition primary with all guards passing)")
    return 0 if winners else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
