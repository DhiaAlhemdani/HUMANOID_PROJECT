#!/usr/bin/env python3
"""Compile and run tests/tuning/slew_microbench.cpp against the header-only
CommandSlewLimiter.h with plain g++ (no ROS/colcon needed). Writes slew_microbench.json
next to this script and exits non-zero if any gate fails.
"""
import json
import pathlib
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        exe = pathlib.Path(tmp) / "slew_microbench"
        cmd = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-I", str(ROOT / "include"),
               str(HERE / "slew_microbench.cpp"), "-o", str(exe)]
        build = subprocess.run(cmd, capture_output=True, text=True)
        if build.returncode != 0:
            print(build.stderr)
            return 2
        run = subprocess.run([str(exe)], capture_output=True, text=True)
    result = json.loads(run.stdout)
    (HERE / "slew_microbench.json").write_text(json.dumps(result, indent=1))
    for name, g in result.items():
        if name != "summary":
            print(f"  [{'ok' if g['ok'] else 'FAIL'}] {name}: {g['value']}")
    s = result["summary"]
    print(f"slew microbench: {s['pass']}/{s['total']} gates")
    return 0 if s["pass"] == s["total"] else 1


if __name__ == "__main__":
    sys.exit(main())
