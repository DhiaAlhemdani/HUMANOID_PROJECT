#!/usr/bin/env python3
"""Score the Phase 1.2 bounce / vertical-smoothness sweep from run_bounce_sweep.sh.

usage: compare_bounce.py <baseline_dir> <candidate_dir> [candidate_dir ...]

Each dir is a run_case.sh result dir (metrics.json + diag.json). For every
candidate, gates against the baseline (docs/LOCOMOTION_IMPROVEMENTS.md #1.2):

  survived, vx_mean >= 0.90x (no degenerate slow walking),
  pelvis_acc_hf <= 0.90x           <- PRIMARY: >=10% cut of 4-40 Hz vertical accel,
  z_sd_mm, plan_z_sd_mm, z_track_err_mm <= 1.10x + slack,
  roll_rms_deg <= 1.15x + 0.05, arm_swing_pp_deg <= 1.20x + 1.0,
  solve_hz >= 0.90x.

Exit 0 iff at least one candidate passes every gate (a winner exists).
Stdlib only.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

# (path, kind, ratio, slack): kind "max" -> v <= ratio*base + slack; "min" -> v >= ratio*base
GUARDS = [
    ("walk.vx_mean", "min", 0.90, 0.0),
    ("walk.pelvis_acc_hf", "max", 0.90, 0.0),      # primary target
    ("walk.z_sd_mm", "max", 1.10, 0.5),
    ("base.plan_z_sd_mm", "max", 1.10, 0.5),
    ("base.z_track_err_mm", "max", 1.10, 0.5),
    ("walk.roll_rms_deg", "max", 1.15, 0.05),
    ("walk.arm_swing_pp_deg", "max", 1.20, 1.0),
    ("solver.solve_hz", "min", 0.90, 0.0),
]


def load(d: Path) -> dict:
    """Flatten metrics.json and diag.json into one path-addressable dict."""
    flat: dict = {}
    met = json.loads((d / "metrics.json").read_text())
    diag = json.loads((d / "diag.json").read_text())
    flat["survived"] = met.get("survived")
    flat["fell_at"] = met.get("fell_at")
    for k, v in (met.get("walk") or {}).items():
        flat[f"walk.{k}"] = v
    for section in ("base", "legs", "arms", "solver"):
        for k, v in (diag.get(section) or {}).items():
            if not isinstance(v, dict):  # skip pd_tau_max etc.
                flat[f"{section}.{k}"] = v
    flat["policy_switch_hz"] = diag.get("policy_switch_hz")
    return flat


def get(d: dict, path: str):
    return d.get(path)


def gates_for(base: dict, cand: dict) -> list[tuple[str, bool, str]]:
    out: list[tuple[str, bool, str]] = []
    out.append(("survived", bool(cand.get("survived")), f"fell_at={cand.get('fell_at')}"))
    for path, kind, ratio, slack in GUARDS:
        b, c = get(base, path), get(cand, path)
        if b is None or c is None:
            out.append((f"guard {path}", False, f"missing (base={b}, cand={c})"))
            continue
        if kind == "max":
            ok, limit = c <= ratio * b + slack, f"{ratio}x + {slack}"
        else:
            ok, limit = c >= ratio * b, f">= {ratio}x"
        out.append((f"guard {path}", ok, f"{b} -> {c} (limit {limit})"))
    return out


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__)
        return 2
    base = load(Path(argv[1]))
    cands = [(p, load(Path(p))) for p in argv[2:]]

    print(f"baseline: {argv[1]}")
    print(f"  acc_hf {get(base, 'walk.pelvis_acc_hf')} | GT z_sd {get(base, 'walk.z_sd_mm')} mm | "
          f"plan_z {get(base, 'base.plan_z_sd_mm')} mm | roll {get(base, 'walk.roll_rms_deg')} deg | "
          f"vx {get(base, 'walk.vx_mean')} | solve {get(base, 'solver.solve_hz')} Hz\n")

    hdr = f"{'candidate':16s} {'acc_hf':>7s} {'GT z_sd':>8s} {'plan_z':>7s} {'roll':>6s} {'swing':>6s} {'vx':>6s}  verdict"
    print(hdr)
    print("-" * len(hdr))

    winners: list[str] = []
    detail: list[tuple[str, list[tuple[str, bool, str]]]] = []
    for path, cand in cands:
        name = Path(path).name
        gates = gates_for(base, cand)
        detail.append((name, gates))
        ok = all(g[1] for g in gates)
        if ok:
            winners.append(name)
        print(f"{name:16s} {get(cand, 'walk.pelvis_acc_hf') or 0:7.3f} "
              f"{get(cand, 'walk.z_sd_mm') or 0:8.1f} {get(cand, 'base.plan_z_sd_mm') or 0:7.2f} "
              f"{get(cand, 'walk.roll_rms_deg') or 0:6.2f} {get(cand, 'walk.arm_swing_pp_deg') or 0:6.1f} "
              f"{get(cand, 'walk.vx_mean') or 0:6.3f}  {'WINNER' if ok else 'no'}")

    print()
    for name, gates in detail:
        for gname, ok, gdet in gates:
            if not ok:
                print(f"  [FAIL] {name}: {gname}: {gdet}")
    print(f"\nwinners: {', '.join(winners) if winners else 'NONE'} "
          f"({len(winners)}/{len(cands)} candidates pass all gates)")
    return 0 if winners else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
