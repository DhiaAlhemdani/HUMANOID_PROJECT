#!/usr/bin/env python3
"""Score an A/B of control.solveBlendSeconds from two run_case.sh result dirs.

usage: compare_blend_ab.py <dir_off> <dir_on>

Reads <dir>/{diag.json, metrics.json} written by plot_tracking.py and applies
PASS/FAIL gates (docs/results/policy_swap_blend.md):

  1. both runs survived;
  2. legs cmd_tau_jump_at_switch drops by >= 40% with blend on;
  3. guard metrics not regressed: plan/bounce z sd, z track error, legs PD peak,
     solve rate, and the non-switch per-tick step (which may rise a little as the
     jump is redistributed over the ramp ticks);
  4. arms jump reported (informational, gates on legs only).

Stdlib only. Exit 0 iff every gate passes. Called by run_blend_ab.sh; can also be
run by hand on any two result dirs.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

# (diag path, max ratio vs off, absolute slack) — regression guards.
GUARDS = [
    ("base.plan_z_sd_mm", 1.10, 1.0),     # bounce: ratio, +1 mm
    ("base.z_track_err_mm", 1.10, 1.0),   # height tracking: ratio, +1 mm
    ("legs.pd_tau_p999", 1.15, 1.0),      # touchdown PD peak: ratio, +1 Nm
    ("solver.solve_hz", 0.90, 0.0),       # solve rate must not drop >10% (ratio <1 = worse)
    ("legs.cmd_tau_step_otherwise", 1.60, 0.05),  # ramp ticks add a little: ratio, +0.05 Nm
]
JUMP_MIN_REDUCTION = 0.60  # on_jump <= 60% of off_jump (>=40% drop)


def get(d: dict, path: str):
    cur = d
    for k in path.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


def load(d: Path) -> tuple[dict, dict]:
    diag = json.loads((d / "diag.json").read_text())
    met = json.loads((d / "metrics.json").read_text())
    return diag, met


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(__doc__)
        return 2
    off_dir, on_dir = Path(argv[1]), Path(argv[2])
    off, off_m = load(off_dir)
    on, on_m = load(on_dir)

    gates: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str) -> None:
        gates.append((name, ok, detail))

    # 1. survival
    for tag, m in (("off", off_m), ("on", on_m)):
        check(f"{tag} survived", bool(m.get("survived")),
              f"fell_at={m.get('fell_at')}")

    # 2. the headline: switch-tick torque step
    j_off = get(off, "legs.cmd_tau_jump_at_switch")
    j_on = get(on, "legs.cmd_tau_jump_at_switch")
    check("legs jump@switch drops >=40%",
          j_off is not None and j_on is not None and j_on <= JUMP_MIN_REDUCTION * j_off,
              f"{j_off} -> {j_on} Nm")
    a_off, a_on = get(off, "arms.cmd_tau_jump_at_switch"), get(on, "arms.cmd_tau_jump_at_switch")
    print(f"info: arms jump@switch {a_off} -> {a_on} Nm (not gated)")

    # 3. guards
    for path, ratio, slack in GUARDS:
        v_off, v_on = get(off, path), get(on, path)
        if v_off is None or v_on is None:
            check(f"guard {path}", False, f"missing (off={v_off}, on={v_on})")
            continue
        if "solve_hz" in path:  # lower is worse for solve rate
            ok = v_on >= ratio * v_off
        else:                   # higher is worse for the rest
            ok = v_on <= ratio * v_off + slack
        check(f"guard {path}", ok, f"{v_off} -> {v_on} (limit {ratio}x + {slack})")

    # 4. context
    print(f"info: policy_switch_hz {get(off, 'policy_switch_hz')} -> {get(on, 'policy_switch_hz')}")
    print(f"info: solver {get(off, 'solver.solve_hz')} -> {get(on, 'solver.solve_hz')} Hz, "
          f"advance p95 {get(off, 'solver.advance_ms_p95')} -> {get(on, 'solver.advance_ms_p95')} ms")

    print(f"\n{'gate':44s} result  detail")
    print("-" * 96)
    failed = 0
    for name, ok, detail in gates:
        print(f"{name:44s} {'PASS' if ok else 'FAIL':7s} {detail}")
        failed += 0 if ok else 1
    verdict = "PASS" if not failed else "FAIL"
    print(f"\nBLEND A/B: {verdict} ({len(gates) - failed}/{len(gates)} gates)")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
