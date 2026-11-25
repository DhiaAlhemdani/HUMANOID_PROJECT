#!/usr/bin/env python3
"""Generic A/B scorer for run_case.sh result dirs (metrics.json + diag.json), replacing the
per-sweep compare_*.py scripts for new rounds.

usage: compare_ab.py [--primary PATH:RATIO[:SLACK]]... [--guard PATH:KIND:RATIO:SLACK]...
                     [--no-default-guards] [--same KEY]... <baseline_dir> <candidate_dir>...

  --primary   any-of: a candidate IMPROVES if value <= RATIO*base + SLACK (SLACK default 0).
              A primary that is not improved must not be worse than 1.10x + SLACK (auto guard).
  --guard     KIND max: value <= RATIO*base + SLACK; KIND min: value >= RATIO*base.
  --same      an integer/str key that must equal the baseline's (e.g. gait.changes_locomotion).
  Default guards (benchmark priorities: falls, then smoothness, then speed) unless --no-default-guards:
    survived; walk.pelvis_acc_hf max 1.10; walk.roll_rms_deg max 1.15+0.05;
    legs.cmd_tau_jump_at_switch max 1.15+0.02; legs.pd_tau_hf max 1.10+0.1;
    legs.track_err_q_deg max 1.20+0.05; base.plan_z_sd_mm max 1.10+0.5; base.z_track_err_mm max 1.10+0.5;
    walk.arm_swing_pp_deg max 1.20+1.0; walk.vx_mean min 0.90; solver.solve_hz min 0.90.
  PATH is section.key: walk.* from metrics.json, everything else from diag.json (base, legs, arms,
  solver, gait, command). WINNER = survived, >=1 primary improved, every guard passed.
  Exit 0 iff a winner exists; load never raises (missing dirs -> ERROR row with the reason).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

DEFAULT_GUARDS = [
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
    flat: dict = {"survived": met.get("survived"), "fell_at": met.get("fell_at")}
    for k, v in (met.get("walk") or {}).items():
        flat[f"walk.{k}"] = v
    for section, body in diag.items():
        if isinstance(body, dict):
            for k, v in body.items():
                if not isinstance(v, (dict, list)):
                    flat[f"{section}.{k}"] = v
        elif not isinstance(body, list):
            flat[section] = body  # top-level scalars (policy_switch_hz, contact_changes_per_s)
    return flat, None


def parse_primary(spec: str):
    parts = spec.split(":")
    if len(parts) not in (2, 3):
        raise SystemExit(f"--primary needs PATH:RATIO[:SLACK], got {spec}")
    return parts[0], float(parts[1]), float(parts[2]) if len(parts) == 3 else 0.0


def parse_guard(spec: str):
    parts = spec.split(":")
    if len(parts) != 4 or parts[1] not in ("max", "min"):
        raise SystemExit(f"--guard needs PATH:max|min:RATIO:SLACK, got {spec}")
    return parts[0], parts[1], float(parts[2]), float(parts[3])


def score(base: dict, cand: dict, primaries, guards, same):
    rows = [("survived", bool(cand.get("survived")), f"fell_at={cand.get('fell_at')}")]
    ok_all = bool(cand.get("survived"))
    for key in same:
        eq = base.get(key) is not None and base.get(key) == cand.get(key)
        ok_all &= eq
        rows.append((f"same {key}", eq, f"base {base.get(key)} vs cand {cand.get(key)}"))
    improved = []
    for path, ratio, slack in primaries:
        b, c = base.get(path), cand.get(path)
        if b is None or c is None:
            rows.append((f"primary {path}", False, f"missing (base={b}, cand={c})"))
            ok_all = False
            continue
        if c <= ratio * b + slack:
            improved.append(path)
            rows.append((f"primary {path}", True, f"{b} -> {c} (improve limit {ratio}x + {slack})"))
        else:
            nw = c <= 1.10 * b + slack
            ok_all &= nw
            rows.append((f"primary {path}", nw, f"{b} -> {c} (not improved; not-worse limit 1.1x + {slack})"))
    for path, kind, ratio, slack in guards:
        b, c = base.get(path), cand.get(path)
        if b is None or c is None:
            rows.append((f"guard {path}", False, f"missing (base={b}, cand={c})"))
            ok_all = False
            continue
        ok = (c <= ratio * b + slack) if kind == "max" else (c >= ratio * b)
        ok_all &= ok
        rows.append((f"guard {path}", ok, f"{b} -> {c} (limit {'<= %gx + %g' % (ratio, slack) if kind == 'max' else '>= %gx' % ratio})"))
    return ok_all and bool(improved), improved, rows


def fmt(v) -> str:
    if isinstance(v, bool) or v is None:
        return f"{str(v):>8s}"
    if isinstance(v, (int, float)):
        return f"{v:8.3f}" if abs(v) < 1000 else f"{v:8.1f}"
    return f"{str(v)[:8]:>8s}"


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--primary", action="append", default=[])
    ap.add_argument("--guard", action="append", default=[])
    ap.add_argument("--same", action="append", default=[])
    ap.add_argument("--no-default-guards", action="store_true")
    ap.add_argument("--show", action="append", default=[], help="extra PATH columns to print")
    ap.add_argument("baseline")
    ap.add_argument("candidates", nargs="+")
    args = ap.parse_args(argv[1:])
    if not args.primary:
        ap.error("at least one --primary is required")
    primaries = [parse_primary(s) for s in args.primary]
    guards = ([] if args.no_default_guards else list(DEFAULT_GUARDS)) + [parse_guard(s) for s in args.guard]
    guards = [g for g in guards if g[0] not in {p[0] for p in primaries}]  # a primary is never also a guard

    base, err = load(Path(args.baseline))
    if base is None:
        print(f"BASELINE FAILED: {args.baseline}: {err}")
        return 2
    cols = [p[0] for p in primaries] + [s for s in args.show if s not in {p[0] for p in primaries}]
    print(f"baseline: {args.baseline}")
    print("  " + " | ".join(f"{c.split('.', 1)[-1]} {base.get(c)}" for c in cols) +
          f" | acc_hf {base.get('walk.pelvis_acc_hf')} | vx {base.get('walk.vx_mean')} | solve {base.get('solver.solve_hz')} Hz\n")
    short = [c.split(".", 1)[-1][:8] for c in cols]
    hdr = f"{'candidate':14s} " + " ".join(f"{s:>8s}" for s in short) + f" {'acc_hf':>8s} {'vx':>8s}  verdict"
    print(hdr)
    print("-" * len(hdr))
    winners, detail = [], []
    cands = [(p, *load(Path(p))) for p in args.candidates]
    for path, cand, cerr in cands:
        name = Path(path).name
        if cand is None:
            detail.append((name, [("load", False, cerr or "unknown error")]))
            print(f"{name:14s} " + " ".join(f"{'-':>8s}" for _ in cols) + f" {'-':>8s} {'-':>8s}  ERROR")
            continue
        wins, improved, rows = score(base, cand, primaries, guards, args.same)
        detail.append((name, rows))
        if wins:
            winners.append(name)
        verdict = ("WINNER (" + ", ".join(p.split(".", 1)[-1] for p in improved) + ")") if wins else (
            "no gain" if all(r[1] for r in rows if not r[0].startswith("primary")) and not improved else "guards-fail")
        print(f"{name:14s} " + " ".join(fmt(cand.get(c)) for c in cols) +
              f" {fmt(cand.get('walk.pelvis_acc_hf'))} {fmt(cand.get('walk.vx_mean'))}  {verdict}")
    print()
    for name, rows in detail:
        for gname, ok, gdet in rows:
            if not ok:
                print(f"  [FAIL] {name}: {gname}: {gdet}")
    print(f"\nwinners: {', '.join(winners) if winners else 'NONE'} ({len(winners)}/{len(cands)} candidates)")
    return 0 if winners else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
