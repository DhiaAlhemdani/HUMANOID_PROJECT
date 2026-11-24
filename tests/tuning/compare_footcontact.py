#!/usr/bin/env python3
"""Score the Phase 1.4 foot/contact-model sweep from run_footcontact_sweep.sh.

usage: compare_footcontact.py <baseline_dir> <candidate_dir> [candidate_dir ...]

Each dir is a run_case.sh result dir (metrics.json + diag.json). Roadmap #1.4 expects the
contact model to help "both bounce and lateral tracking" and the D2 peak sits at the contact
layer, so this round has ANY-OF primaries (a candidate wins if every guard passes and at least
one primary clears its threshold):

  PRIMARIES  walk.pelvis_acc_hf <= 0.92x   (bounce, benchmark priority #1 after falls)
             walk.roll_rms_deg  <= 0.85x   (lateral)
             legs.pd_tau_p999   <= 0.90x   (D2 touchdown peak)
  GUARDS     survived; every primary not improved must not worsen (acc_hf <= 1.10x,
             roll <= 1.15x + 0.05, peak <= 1.05x); legs.cmd_tau_jump_at_switch <= 1.15x + 0.02,
             leg PD HF <= 1.10x + 0.1, legs.track_err_q_deg <= 1.20x + 0.05,
             plan z sd / z-track <= 1.10x + 0.5, arm swing <= 1.20x + 1.0,
             vx >= 0.90x, solve_hz >= 0.90x.

Exit 0 iff at least one candidate wins. Stdlib only; load() never raises.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

# (path, ratio): primary improves iff v <= ratio*base
PRIMARIES = [
    ("walk.pelvis_acc_hf", 0.92),
    ("walk.roll_rms_deg", 0.85),
    ("legs.pd_tau_p999", 0.90),
]
# (path, kind, ratio, slack): kind "max" -> v <= ratio*base + slack; "min" -> v >= ratio*base
GUARDS = [
    ("walk.pelvis_acc_hf", "max", 1.10, 0.0),
    ("walk.roll_rms_deg", "max", 1.15, 0.05),
    ("legs.pd_tau_p999", "max", 1.05, 0.0),
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
    """Return (flat_metrics, None) or (None, reason); never raises."""
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
    flat: dict = {"survived": met.get("survived"), "fell_at": met.get("fell_at")}
    for k, v in (met.get("walk") or {}).items():
        flat[f"walk.{k}"] = v
    for section in ("base", "legs", "arms", "solver"):
        for k, v in (diag.get(section) or {}).items():
            if not isinstance(v, dict):
                flat[f"{section}.{k}"] = v
    return flat, None


def score(base: dict, cand: dict) -> tuple[bool, list[str], list[tuple[str, bool, str]]]:
    """-> (wins, improved_primaries, gate rows [(name, ok, detail)])."""
    rows: list[tuple[str, bool, str]] = [
        ("survived", bool(cand.get("survived")), f"fell_at={cand.get('fell_at')}")]
    improved: list[str] = []
    for path, ratio in PRIMARIES:
        b, c = base.get(path), cand.get(path)
        if b is None or c is None:
            rows.append((f"primary {path}", False, f"missing (base={b}, cand={c})"))
            continue
        ok = c <= ratio * b
        if ok:
            improved.append(path)
        rows.append((f"primary {path}", ok, f"{b} -> {c} (improve limit {ratio}x)"))
    guards_ok = True
    for path, kind, ratio, slack in GUARDS:
        b, c = base.get(path), cand.get(path)
        if b is None or c is None:
            rows.append((f"guard {path}", False, f"missing (base={b}, cand={c})"))
            guards_ok = False
            continue
        if kind == "max":
            ok, limit = c <= ratio * b + slack, f"{ratio}x + {slack}"
        else:
            ok, limit = c >= ratio * b, f">= {ratio}x"
        guards_ok &= ok
        rows.append((f"guard {path}", ok, f"{b} -> {c} (limit {limit})"))
    wins = bool(cand.get("survived")) and guards_ok and bool(improved)
    return wins, improved, rows


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__)
        return 2
    base, base_err = load(Path(argv[1]))
    if base is None:
        print(f"BASELINE FAILED: {argv[1]}: {base_err}")
        return 2

    print(f"baseline: {argv[1]}")
    print(f"  acc_hf {base.get('walk.pelvis_acc_hf')} | roll {base.get('walk.roll_rms_deg')} | "
          f"PD peak {base.get('legs.pd_tau_p999')} | jump@switch {base.get('legs.cmd_tau_jump_at_switch')} | "
          f"z_sd {base.get('base.plan_z_sd_mm')} | vx {base.get('walk.vx_mean')} | "
          f"solve {base.get('solver.solve_hz')} Hz\n")

    hdr = (f"{'candidate':14s} {'acc_hf':>7s} {'roll':>6s} {'PDpeak':>7s} {'jump':>6s} {'PDHF':>6s} "
           f"{'qerr':>6s} {'z_sd':>5s} {'vx':>6s}  verdict")
    print(hdr)
    print("-" * len(hdr))

    winners: list[str] = []
    detail: list[tuple[str, list[tuple[str, bool, str]]]] = []
    cands = [(p, *load(Path(p))) for p in argv[2:]]
    for path, cand, cand_err in cands:
        name = Path(path).name
        if cand is None:
            detail.append((name, [("load", False, cand_err or "unknown error")]))
            print(f"{name:14s} {'-':>7s} {'-':>6s} {'-':>7s} {'-':>6s} {'-':>6s} {'-':>6s} {'-':>5s} {'-':>6s}  ERROR")
            continue
        wins, improved, rows = score(base, cand)
        detail.append((name, [r for r in rows if r[0].startswith("guard") or r[0] == "survived"]))
        if wins:
            winners.append(name)
        verdict = ("WINNER (" + ", ".join(p.split(".")[-1] for p in improved) + ")") if wins else (
            "guards-fail" if not all(r[1] for r in rows if not r[0].startswith("primary")) else "no gain")
        print(f"{name:14s} {num(cand, 'walk.pelvis_acc_hf'):7.3f} {num(cand, 'walk.roll_rms_deg'):6.2f} "
              f"{num(cand, 'legs.pd_tau_p999'):7.2f} {num(cand, 'legs.cmd_tau_jump_at_switch'):6.3f} "
              f"{num(cand, 'legs.pd_tau_hf'):6.2f} {num(cand, 'legs.track_err_q_deg'):6.3f} "
              f"{num(cand, 'base.plan_z_sd_mm'):5.1f} {num(cand, 'walk.vx_mean'):6.3f}  {verdict}")

    print()
    for name, rows in detail:
        for gname, ok, gdet in rows:
            if not ok:
                print(f"  [FAIL] {name}: {gname}: {gdet}")
    print(f"\nwinners: {', '.join(winners) if winners else 'NONE'} "
          f"({len(winners)}/{len(cands)} candidates improve a primary with all guards passing)")
    return 0 if winners else 1


def num(d: dict, path: str) -> float:
    v = d.get(path)
    return float(v) if isinstance(v, (int, float)) else 0.0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
