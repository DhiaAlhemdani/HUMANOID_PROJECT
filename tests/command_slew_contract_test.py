"""Phase 1.6 contract: the S-curve command slew limiter is wired end to end and inert at 0.

Static checks on the sources (no build needed) + the standalone g++ microbench when g++ exists
+ the `command` transient section of plot_tracking.py on a synthetic run (needs numpy/matplotlib).
"""

import importlib.util
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HDR = ROOT / "include/legged_robot_mpc_controller/mpc/common/target/CommandSlewLimiter.h"
TARGET_CPP = ROOT / "src/mpc/common/target/WalkingVelocityTarget.cpp"
TARGET_H = ROOT / "include/legged_robot_mpc_controller/mpc/common/target/WalkingVelocityTarget.h"
BASE_H = ROOT / "include/legged_robot_mpc_controller/mpc/common/command/TargetTrajectoriesCalculatorBase.h"
CTRL = ROOT / "src/controller/controller.cpp"
PARAMS = ROOT / "src/controller/parameters.yaml"
G1 = ROOT / "config/g1/ros2_controllers.yaml"
HAVE_PLOT = importlib.util.find_spec("numpy") is not None and importlib.util.find_spec("matplotlib") is not None


class Wiring(unittest.TestCase):
    def test_header_is_dependency_free_and_marked(self):
        text = HDR.read_text()
        includes = re.findall(r"^#include\s*[<\"]([^>\"]+)[>\"]", text, flags=re.M)
        self.assertEqual(sorted(includes), ["algorithm", "cmath"])  # standalone: no ocs2/Eigen
        self.assertIn("// SLEW_LIMITER_BEGIN", text)
        self.assertIn("// SLEW_LIMITER_END", text)
        self.assertIn("inline double slewLimitStep(", text)

    def test_reference_config_and_target_use_it(self):
        self.assertIn("vector4_t commandJerkLimits{vector4_t::Zero()};", BASE_H.read_text())
        h = TARGET_H.read_text()
        self.assertIn("CommandSlewLimiter.h", h)
        self.assertIn("vector4_t commandJerkLimits_;", h)
        self.assertIn("vector4_t previousRate_{vector4_t::Zero()};", h)
        cpp = TARGET_CPP.read_text()
        self.assertIn("commandJerkLimits_(referenceConfig.commandJerkLimits)", cpp)
        self.assertIn("slewLimitStep(unlimited, previousCommand_(i), previousRate_(i), dt, rateLimit, jerkLimit)", cpp)
        self.assertIn("previousRate_.setZero();", cpp)      # reset with the filter
        self.assertNotIn("std::clamp(unlimited, previousCommand_(i) - increment", cpp)  # old clamp gone
        # the takeover height rise keeps the plain clamp
        self.assertRegex(cpp, r"risingFromResetHeight_\) \? 0\.0 : commandJerkLimits_\(i\)")

    def test_parameter_plumbing(self):
        self.assertIn("parameters_.ocs2.reference.velocityCommandJerkLimits.data()", CTRL.read_text())
        p = PARAMS.read_text()
        m = re.search(r"velocityCommandJerkLimits: \{type: double_array, default_value: \[([^\]]*)\]", p)
        self.assertIsNotNone(m)
        self.assertEqual([float(x) for x in m.group(1).split(",")], [0.0, 0.0, 0.0, 0.0])
        self.assertIn("fixed_size<>: [4]", p[m.start():m.start() + 600])
        g1 = G1.read_text()
        m = re.search(r"velocityCommandJerkLimits:\s*\n\s*\[([^\]]*)\]", g1)
        self.assertIsNotNone(m, "g1 config must carry the key (shipped inert)")
        self.assertEqual([float(x) for x in m.group(1).split(",")], [0.0, 0.0, 0.0, 0.0])

    @unittest.skipIf(shutil.which("g++") is None, "g++ not available")
    def test_microbench_gates(self):
        r = subprocess.run([sys.executable, str(ROOT / "tests/tuning/run_slew_microbench.py")],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        res = json.loads((ROOT / "tests/tuning/slew_microbench.json").read_text())
        self.assertEqual(res["summary"]["pass"], res["summary"]["total"])
        self.assertTrue(res["legacy_rate_equals_limit"]["ok"])   # jerk 0 == old behaviour
        self.assertTrue(res["scurve_jerk_within_limit"]["ok"])


@unittest.skipUnless(HAVE_PLOT, "numpy/matplotlib not installed")
class CommandTransientSection(unittest.TestCase):
    def test_vx_profile_events_at_probe_schedule(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp) / "run"
            subprocess.run([sys.executable, str(ROOT / "tests/tuning/synth_run.py"), str(d), "--sequence", "walk:20",
                            "--bump", "0.0"], check=True, capture_output=True)
            subprocess.run([sys.executable, str(ROOT / "tests/tuning/plot_tracking.py"), str(d), "run"],
                           check=True, capture_output=True, cwd=tmp)
            cmd = json.loads((d / "diag.json").read_text())["command"]
        self.assertEqual(cmd["profile"], "vx")
        self.assertEqual([e["kind"] for e in cmd["events"]],
                         ["ramp_up_start", "ramp_up_end", "ramp_down_start", "ramp_down_end"])
        self.assertEqual([e["t"] for e in cmd["events"]], [4.0, 7.0, 27.0, 30.0])  # SETTLE, +RAMP, +HOLD, +RAMP
        for e in cmd["events"]:
            for k in ("acc_hf", "z_pp_mm", "pitch_max_deg", "roll_max_deg"):
                self.assertIn(k, e)
        self.assertIsNotNone(cmd["transient_pitch_max_deg"])


if __name__ == "__main__":
    unittest.main()
