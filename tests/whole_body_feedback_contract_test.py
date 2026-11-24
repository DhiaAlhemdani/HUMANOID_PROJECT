#!/usr/bin/env python3
"""Phase 2.B S1 whole-body feedback layer: schema / shipped config / wiring / sweep contract.

Runs without ROS. The C++ core has its own Eigen-only regression (whole_body_feedback_core_test.cpp);
this pins what the sweep and the docs rely on: the block ships disabled with every key the
schema defines, the controller reads each key, the layer is built into the library, and the
sweep script only sets keys that exist.
"""
import pathlib
import re
import unittest

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
CTRL = "humanoid_whole_body_inverse_dynamics_mpc_controller"


def schema_whole_body():
    schema = yaml.safe_load((ROOT / "src/controller/parameters.yaml").read_text())
    return next(iter(schema.values()))["control"]["wholeBody"]


class WholeBodyFeedbackContract(unittest.TestCase):
    def test_shipped_g1_block_is_inert_and_complete(self):
        schema = schema_whole_body()
        g1 = yaml.safe_load((ROOT / "config/g1/ros2_controllers.yaml").read_text())
        block = g1[CTRL]["ros__parameters"]["control"]["wholeBody"]
        self.assertFalse(block["enabled"])
        self.assertFalse(schema["enabled"]["default_value"])
        self.assertEqual(set(block), set(schema), "G1 block must carry every schema key (make_config edits in place)")
        for key, spec in schema.items():
            if key == "enabled":
                continue
            self.assertEqual(block[key], spec["default_value"], f"{key}: shipped value differs from the schema default")
        for key in ("baseKp", "baseKd", "baseWeight"):
            self.assertEqual(len(block[key]), 6)
        for key in ("swingKp", "swingKd"):
            self.assertEqual(len(block[key]), 3)
        # Horizontal base position is left to the MPC (no fight with its footholds / velocity tracking).
        self.assertEqual(block["baseKp"][:2], [0.0, 0.0])

    def test_h2_untouched(self):
        h2 = yaml.safe_load((ROOT / "config/h2/ros2_controllers.yaml").read_text())
        self.assertNotIn("wholeBody", h2[CTRL]["ros__parameters"]["control"])

    def test_controller_reads_every_key(self):
        src = (ROOT / "src/controller/controller.cpp").read_text()
        for key in schema_whole_body():
            self.assertIn(f"wholeBody.{key}" if key == "enabled" or key == "gainRampSeconds" else f"wb.{key}", src,
                          f"controller never reads control.wholeBody.{key}")
        # The layer only replaces the MPC locomotion torque, never bring-up commands, and the
        # residual gains engage in Locomotion only.
        self.assertIn("BringUpPhase::Locomotion", src.split("update_whole_body_gain_ramp(double now)")[1].split("\n}\n")[0])
        self.assertIn("whole_body_feedback_->compute(*control_pinocchio_, input)", src)

    def test_library_and_test_wired(self):
        cmake = (ROOT / "CMakeLists.txt").read_text()
        self.assertIn("src/controller/whole_body_feedback.cpp", cmake)
        self.assertIn("whole_body_feedback_core_test", cmake)
        self.assertTrue((ROOT / "tests/whole_body_feedback_core_test.cpp").is_file())
        core = (ROOT / "include/legged_robot_mpc_controller/controller/whole_body_feedback_core.hpp").read_text()
        self.assertNotIn("#include <pinocchio", core, "the core must stay Eigen-only (sandbox-testable)")

    def test_sweep_sets_only_existing_keys(self):
        script = (ROOT / "tests/tuning/run_wbc_sweep.sh").read_text()
        keys = set(schema_whole_body())
        for key in re.findall(r"\$WB\.(\w+)=", script):
            self.assertIn(key, keys, f"sweep sets unknown key control.wholeBody.{key}")
        self.assertIn("meas_tau_hf", (ROOT / "tests/tuning/plot_tracking.py").read_text())
        self.assertIn("--no-default-guards", script)

    def test_tracking_columns(self):
        src = (ROOT / "src/controller/controller.cpp").read_text()
        for name in ("active", "residual", "clips", "gain_alpha"):
            self.assertIn(f'"{name}"', src)


if __name__ == "__main__":
    unittest.main()
