#!/usr/bin/env python3
"""Micro-benchmark for the double-support load-share cost (structural D2/D3 lever).

Extracts the marked load_share free-function block VERBATIM from
include/legged_robot_mpc_controller/mpc/wb_id/cost/WbIdContactLoadShareCost.h,
compiles it with tests/tuning/loadshare_microbench.cpp (synthetic walk schedule +
gates), runs it and writes tests/tuning/loadshare_microbench.json.

Usage:  python3 tests/tuning/run_loadshare_microbench.py
Needs only: g++, python3. No ROS, no MuJoCo.

Implements the 'test' step of the implement -> test -> document methodology for
docs/LOCOMOTION_IMPROVEMENTS.md #1.3 (force-rate / D2-D3 structural round).
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HEADER = REPO / "include" / "legged_robot_mpc_controller" / "mpc" / "wb_id" / "cost" / "WbIdContactLoadShareCost.h"
HARNESS = REPO / "tests" / "tuning" / "loadshare_microbench.cpp"
OUT_JSON = REPO / "tests" / "tuning" / "loadshare_microbench.json"

BEGIN = "[load_share_free_functions_begin]"
END = "[load_share_free_functions_end]"


def extract_free_functions() -> str:
    text = HEADER.read_text()
    if text.count(BEGIN) != 1 or text.count(END) != 1:
        raise SystemExit(f"expected exactly one extraction marker pair in {HEADER}")
    block = text.split(BEGIN, 1)[1].split(END, 1)[0]
    if "namespace load_share" not in block:
        raise SystemExit("extracted block does not contain namespace load_share")
    return block


def main() -> int:
    if shutil.which("g++") is None:
        print("g++ not found", file=sys.stderr)
        return 2
    inc = extract_free_functions()
    with tempfile.TemporaryDirectory(prefix="loadshare_mb_") as tmp:
        tmpdir = Path(tmp)
        (tmpdir / "loadshare_free.inc").write_text(inc)
        binary = tmpdir / "loadshare_microbench"
        compile_cmd = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-I", str(tmpdir),
                       str(HARNESS), "-o", str(binary)]
        proc = subprocess.run(compile_cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            print(proc.stdout)
            print(proc.stderr, file=sys.stderr)
            print("loadshare microbench: COMPILE FAILED", file=sys.stderr)
            return 1
        run = subprocess.run([str(binary)], capture_output=True, text=True)
        sys.stdout.write(run.stdout)
        if run.stderr:
            sys.stderr.write(run.stderr)

    gates = dict(re.findall(r"GATE (\w+): (PASS|FAIL)", run.stdout))
    passed = sum(1 for v in gates.values() if v == "PASS")
    total = len(gates)
    result = {
        "harness": str(HARNESS.relative_to(REPO)),
        "source": str(HEADER.relative_to(REPO)),
        "gates": gates,
        "passed": passed,
        "total": total,
    }
    OUT_JSON.write_text(json.dumps(result, indent=2) + "\n")
    print(f"loadshare microbench: {passed}/{total} gates -> {OUT_JSON.relative_to(REPO)}")
    if run.returncode != 0 or passed != total:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
