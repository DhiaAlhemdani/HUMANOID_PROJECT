#!/usr/bin/env python3
"""Micro-benchmark runner for the solve-handover command blend (Phase 1.1 measurement).

Extracts apply_policy_swap_blend() VERBATIM from src/controller/controller.cpp,
compiles it with tests/tuning/blend_microbench.cpp (stub types + synthetic 1 kHz
trace calibrated to the documented 0.79 / 0.31 Nm walk measurements), runs the
solve-rate x blend-duration sweep, applies PASS/FAIL gates, prints a table and
writes tests/tuning/blend_microbench.json.

Usage:  python3 tests/tuning/run_blend_microbench.py
Needs only: g++, python3. No ROS, no MuJoCo -- for the in-container A/B use
tests/tuning/run_blend_ab.sh instead.

Implements the 'test' step of the adopt-implement-test-document methodology for
control.solveBlendSeconds (docs/LOCOMOTION_IMPROVEMENTS.md #1.1).
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SRC = REPO / "src" / "controller" / "controller.cpp"
HARNESS = REPO / "tests" / "tuning" / "blend_microbench.cpp"
OUT_JSON = REPO / "tests" / "tuning" / "blend_microbench.json"

# Gates (Nm unless noted). Rationale in docs/results/policy_swap_blend.md.
GATE_JUMP_ON = 1e-6        # blend must make the switch-tick step exactly 0 (re-anchor)
GATE_LAG_AFTER = 1e-9      # after the ramp the command equals the new policy (no lag)
GATE_MAX_STEP_MARGIN = 1e-9  # blended worst per-tick step must beat its baseline


def extract_blend_function() -> str:
    """Return the verbatim apply_policy_swap_blend() definition from controller.cpp."""
    text = SRC.read_text()
    sig = re.search(
        r"HumanoidWholeBodyInverseDynamicsMpcController::JointActionCommand\s*\n"
        r"HumanoidWholeBodyInverseDynamicsMpcController::apply_policy_swap_blend\s*\(",
        text,
    )
    if not sig:
        raise SystemExit(f"apply_policy_swap_blend definition not found in {SRC}")
    if text.count("apply_policy_swap_blend(") != 2:  # 1 definition + 1 call site
        raise SystemExit("unexpected apply_policy_swap_blend occurrence count")
    body_start = text.index("{", sig.end())
    depth, i = 0, body_start
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                break
        i += 1
    if depth != 0:
        raise SystemExit("unbalanced braces in apply_policy_swap_blend")
    return text[sig.start() : i + 1]


def build_and_run(inc_path: Path) -> dict:
    bin_path = inc_path.with_suffix(".bin")
    cmd = [
        "g++", "-O2", "-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror",
        f'-DBLEND_EXTRACTED_INC="{inc_path}"',
        str(HARNESS), "-o", str(bin_path),
    ]
    subprocess.run(cmd, check=True)
    proc = subprocess.run([str(bin_path), str(inc_path)], capture_output=True, text=True)
    if proc.stderr:
        sys.stderr.write(proc.stderr)
    if proc.returncode == 3:
        raise SystemExit(f"calibration failed (targets 0.79/0.31 not reproduced):\n{proc.stderr}")
    if proc.returncode != 0:
        raise SystemExit(f"micro-benchmark failed rc={proc.returncode}\n{proc.stderr}\n{proc.stdout}")
    return json.loads(proc.stdout)


def apply_gates(data: dict) -> list[tuple[str, bool, str]]:
    cases = {c["label"]: c for c in data["cases"]}
    gates: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str) -> None:
        gates.append((name, ok, detail))

    check("calibration", data["calibration"]["ok"],
          f"off={data['calibration']['off_measured']:.3f} (t 0.31) "
          f"on={data['calibration']['on_measured']:.3f} (t 0.79)")

    pairs = [  # (baseline label, blend label, overlap?)
        ("58.8 Hz baseline", "58.8 Hz + 5 ms", False),
        ("90.9 Hz baseline", "90.9 Hz + 5 ms (G1 shipped)", False),
        ("100 Hz baseline", "100 Hz + 5 ms", False),
        ("200 Hz baseline", "200 Hz + 2 ms (Upkie shipped)", False),
        ("200 Hz baseline", "200 Hz + 5 ms (overlap stress)", True),
        ("90.9 Hz jittered baseline", "90.9 Hz jittered +-2 ms + 5 ms", False),
    ]
    for base_label, blend_label, overlap in pairs:
        base, blend = cases[base_label], cases[blend_label]
        tag = blend_label
        check(f"{tag}: jump@switch -> 0",
              blend["jump_on"] <= GATE_JUMP_ON,
              f"{blend['jump_on']:.4f} vs baseline {base['jump_on']:.4f} Nm")
        check(f"{tag}: worst per-tick step improves",
              blend["max_step"] < base["max_step"] - GATE_MAX_STEP_MARGIN,
              f"{blend['max_step']:.4f} vs baseline {base['max_step']:.4f} Nm")
        if not overlap:
            check(f"{tag}: no residual lag after ramp",
                  blend["lag_after_ramp_max"] <= GATE_LAG_AFTER,
                  f"max |cmd-policy| after ramp = {blend['lag_after_ramp_max']:.3e} Nm")
    return gates


def main() -> int:
    if not shutil.which("g++"):
        raise SystemExit("g++ not found")
    inc_path = REPO / "tests" / "tuning" / ".blend_extracted.inc"
    inc_path.write_text(extract_blend_function() + "\n")
    try:
        data = build_and_run(inc_path)
    finally:
        inc_path.unlink(missing_ok=True)
        inc_path.with_suffix(".bin").unlink(missing_ok=True)

    gates = apply_gates(data)
    data["gates"] = [{"name": n, "pass": ok, "detail": d} for n, ok, d in gates]
    OUT_JSON.write_text(json.dumps(data, indent=1) + "\n")

    cal = data["calibration"]
    print(f"calibration: off={cal['off_measured']:.4f} (target {cal['off_target']:.2f}), "
          f"on={cal['on_measured']:.4f} (target {cal['on_target']:.2f})")
    hdr = f"{'case':40s} {'jump@switch':>11s} {'step_other':>11s} {'max/tick':>9s} {'lag_after':>10s}"
    print(hdr)
    print("-" * len(hdr))
    for c in data["cases"]:
        print(f"{c['label']:40s} {c['jump_on']:11.4f} {c['step_off']:11.4f} "
              f"{c['max_step']:9.4f} {c['lag_after_ramp_max']:10.1e}")
    print()
    failed = 0
    for name, ok, detail in gates:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
        failed += 0 if ok else 1
    print(f"\n{len(gates) - failed}/{len(gates)} gates passed; results -> {OUT_JSON.relative_to(REPO)}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
