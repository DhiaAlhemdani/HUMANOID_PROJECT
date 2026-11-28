#!/usr/bin/env python3
"""Score the Phase 1.3 structural (D2/D3) load-share sweep from run_loadshare_sweep.sh.

usage: compare_loadshare.py <baseline_dir> <candidate_dir> [candidate_dir ...]

Each dir is a run_case.sh result dir (metrics.json + diag.json). For every candidate,
gates against the baseline (docs/LOCOMOTION_IMPROVEMENTS.md #1.3 structural round):

  survived,
  legs.cmd_tau_jump_at_switch <= 0.85x   <- PRIMARY (D3: per-solve command ripple),
  legs.pd_tau_p999 <= 1.05x              (D2 touchdown peak: no worsening; the sweep
                                          also reports its movement),
  pelvis_acc_hf <= 1.10x, leg PD HF <= 1.10x + 0.1,
  legs.track_err_q_deg <= 1.20x + 0.05, plan z sd / z-track <= 1.10x + 0.5,
  roll <= 1.15x + 0.05, arm swing <= 1.20x + 1.0,
  vx >= 0.90x, solve_hz >= 0.90x.

Exit 0 iff at least one candidate passes every gate. Stdlib only.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

# (path, kind, ratio, slack): kind "max" -> v <= ratio*base + slack; "min" -> v >= ratio*base
PRIMARY = ("legs.cmd_tau_jump_at_switch", "max", 0.85, 0.0)
GUARDS = [
    ("legs.pd_tau_p999", "max", 1.05, 0.0),          # D2 peak: must not worsen
    ("walk.pelvis_acc_hf", "max", 1.10, 0.0),
    ("legs.pd_tau_hf", "max", 1.10, 0.1),
    ("legs.track_err_q_deg", "max", 1.20, 0.05),
    ("base.plan_z_sd_mm", "max", 1.10, 0.5),
    ("base.z_track_err_mm", "max", 1.10, 0.5),
    ("walk.roll_rms_deg", "max", 1.15, 0.05),
    ("walk.arm_swing_pp_deg", "max", 1.20, 1.0),
    ("walk.vx_mean", "min", 0.90, 0.0),
    ("solver.solve_hz", "min", 0.90, 0.0),
]


def load(d: Path) -> tuple[dict | None, str | None]:
    """Return (flat_metrics, None) or (None, reason); never raises — a silent crash
    here produced an empty tee'd report once already."""
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
    flat: dict = {}
    flat["survived"] = met.get("survived")
    flat["fell_at"] = met.get("fell_at")
    for k, v in (met.get("walk") or {}).items():
        flat[f"walk.{k}"] = v
    for section in ("base", "legs", "arms", "solver"):
        for k, v in (diag.get(section) or {}).items():
            if not isinstance(v, dict):
                flat[f"{section}.{k}"] = v
    return flat, None


def gate_list(base: dict, cand: dict) -> list[tuple[str, bool, str]]:
    out: list[tuple[str, bool, str]] = [
        ("survived", bool(cand.get("survived")), f"fell_at={cand.get('fell_at')}")]
    for path, kind, ratio, slack in (PRIMARY, *GUARDS):
        b, c = base.get(path), cand.get(path)
        if b is None or c is None:
            out.append((f"{'primary ' if (path, kind, ratio, slack) == PRIMARY else 'guard '}{path}", False,
                        f"missing (base={b}, cand={c})"))
            continue
        if kind == "max":
            ok, limit = c <= ratio * b + slack, f"{ratio}x + {slack}"
        else:
            ok, limit = c >= ratio * b, f">= {ratio}x"
        label = "primary " if (path, kind, ratio, slack) == PRIMARY else "guard "
        out.append((label + path, ok, f"{b} -> {c} (limit {limit})"))
    return out


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__)
        return 2
    base, base_err = load(Path(argv[1]))
    if base is None:
        print(f"BASELINE FAILED: {argv[1]}: {base_err}")
        return 2

    print(f"baseline: {argv[1]}")
    print(f"  jump@switch {get(base, 'legs.cmd_tau_jump_at_switch')} | PD peak {get(base, 'legs.pd_tau_p999')} | "
          f"acc_hf {get(base, 'walk.pelvis_acc_hf')} | PD HF {get(base, 'legs.pd_tau_hf')} | "
          f"vx {get(base, 'walk.vx_mean')} | solve {get(base, 'solver.solve_hz')} Hz\n")

    hdr = (f"{'candidate':12s} {'jump':>6s} {'PDpeak':>7s} {'acc_hf':>7s} {'PDHF':>6s} {'qerr':>6s} "
           f"{'roll':>6s} {'vx':>6s}  verdict")
    print(hdr)
    print("-" * len(hdr))

    winners: list[str] = []
    detail: list[tuple[str, list[tuple[str, bool, str]]]] = []
    cands = [(p, *load(Path(p))) for p in argv[2:]]
    for path, cand, cand_err in cands:
        name = Path(path).name
        if cand is None:
            detail.append((name, [("load", False, cand_err or "unknown error")]))
            print(f"{name:12s} {'-':>6s} {'-':>7s} {'-':>7s} {'-':>6s} {'-':>6s} {'-':>6s} {'-':>6s}  ERROR")
            continue
        gates = gate_list(base, cand)
        detail.append((name, gates))
        ok = all(g[1] for g in gates)
        if ok:
            winners.append(name)
        print(f"{name:12s} {num(cand, 'legs.cmd_tau_jump_at_switch'):6.3f} {num(cand, 'legs.pd_tau_p999'):7.2f} "
              f"{num(cand, 'walk.pelvis_acc_hf'):7.3f} {num(cand, 'legs.pd_tau_hf'):6.2f} "
              f"{num(cand, 'legs.track_err_q_deg'):6.3f} {num(cand, 'walk.roll_rms_deg'):6.2f} "
              f"{num(cand, 'walk.vx_mean'):6.3f}  {'WINNER' if ok else 'no'}")

    print()
    for name, gates in detail:
        for gname, ok, gdet in gates:
            if not ok:
                print(f"  [FAIL] {name}: {gname}: {gdet}")
    print(f"\nwinners: {', '.join(winners) if winners else 'NONE'} "
          f"({len(winners)}/{len(cands)} candidates pass all gates)")
    return 0 if winners else 1


def get(d: dict, path: str):
    return d.get(path)


def num(d: dict, path: str) -> float:
    v = d.get(path)
    return float(v) if isinstance(v, (int, float)) else 0.0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
