#!/usr/bin/env python3
"""Phase 2.B S1 whole-body feedback layer: schema / shipped config / wiring / sweep contract.

Runs without ROS. The C++ core has its own Eigen-only regression (whole_body_feedback_core_test.cpp);
this pins what the sweep and the docs rely on: the G1 block ships ON (sweep 3 winner: soft set +
30 ms contact-switch fade) with every key the schema defines at the schema default, the library
default stays off (other robots), the controller reads each key, the layer is built into the library, and the
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
    def test_shipped_g1_block_is_on_at_schema_defaults_and_complete(self):
        schema = schema_whole_body()
        g1 = yaml.safe_load((ROOT / "config/g1/ros2_controllers.yaml").read_text())
        block = g1[CTRL]["ros__parameters"]["control"]["wholeBody"]
        self.assertTrue(block["enabled"], "2.B sweep 3 shipped the layer on for G1 (docs/results/wbc_s1_sweep.md)")
        self.assertFalse(schema["enabled"]["default_value"], "library default stays off (H2/Upkie carry no block)")
        self.assertAlmostEqual(block["contactBlendSeconds"], 0.03)
        # The layer writes the residual servo gains: the shipped config must carry jointEffortPd.
        self.assertGreater(max(g1[CTRL]["ros__parameters"]["control"]["jointEffortPd"]["kp"]), 0.0)
        self.assertEqual(set(block), set(schema), "G1 block must carry every schema key (make_config edits in place)")
        names = g1[CTRL]["ros__parameters"]["robot"]["jointNames"]
        for key, spec in schema.items():
            if key == "enabled":
                continue
            if key == "jointWeights":
                # Schema default [] (= uniform) cannot be written in a ROS parameter file (see test below).
                self.assertEqual(block[key], [1.0] * len(names))
                continue
            self.assertEqual(block[key], spec["default_value"], f"{key}: shipped value differs from the schema default")
        for key in ("baseKp", "baseKd", "baseWeight"):
            self.assertEqual(len(block[key]), 6)
        for key in ("swingKp", "swingKd"):
            self.assertEqual(len(block[key]), 3)
        # Horizontal base position is left to the MPC (no fight with its footholds / velocity tracking).
        self.assertEqual(block["baseKp"][:2], [0.0, 0.0])

    def test_touchdown_search_is_a_candidate_and_wired_through(self):
        # Run 29: off in the schema and the shipped G1 block, on (0.3 s) in config/tuning/humanT29.yaml, which is
        # otherwise the shipped recipe (humanT25c.yaml); the layer decides it in the Eigen-only core, the controller
        # forces the fade off while a foot searches, restarts it at the real touchdown and tells the estimator.
        schema = schema_whole_body()
        self.assertEqual(schema["searchWindow"]["default_value"], 0.0)
        g1 = yaml.safe_load((ROOT / "config/g1/ros2_controllers.yaml").read_text())
        self.assertEqual(g1[CTRL]["ros__parameters"]["control"]["wholeBody"]["searchWindow"], 0.0)
        base = yaml.safe_load((ROOT / "config/tuning/humanT25c.yaml").read_text())[CTRL]["ros__parameters"]
        cand = yaml.safe_load((ROOT / "config/tuning/humanT29.yaml").read_text())[CTRL]["ros__parameters"]
        self.assertEqual(cand["control"]["wholeBody"]["searchWindow"], 0.3)
        self.assertEqual(cand["control"]["wholeBody"]["searchLandedVelocity"], 0.05)
        self.assertEqual(cand["control"]["wholeBody"]["searchDescentVelocity"], 0.15)
        cand["control"]["wholeBody"]["searchWindow"] = 0.0
        self.assertEqual(base, cand)
        core = (ROOT / "include/legged_robot_mpc_controller/controller/whole_body_feedback_core.hpp").read_text()
        self.assertIn("inline bool touchdownSearch(const TouchdownSearchSettings& s,", core)
        layer = (ROOT / "src/controller/whole_body_feedback.cpp").read_text()
        self.assertIn("touchdownSearch(settings_.search, in.time, in.inContact[c], in.inContact[1 - c], footVel[c].z()", layer)
        self.assertIn("inContact[c] = in.inContact[c] && !out.searching[c];", layer)
        self.assertIn("p.accWeight(col) *= kSearchLegWeight;", layer)  # run 29 B: the searching leg is free of the posture PD
        src = (ROOT / "src/controller/controller.cpp").read_text()
        for needle in ("input.time = observation.time;",
                       "if ((feedforward_is_fallback && wb.holdOnTorqueWindowOverrun) || searching) {",
                       "whole_body_last_switch_time_ = policy_time;\n      }\n      whole_body_contact_alpha_ = contact_switch_fade(",
                       "contact[c] = contact[c] && !whole_body_searching_[c];",
                       '"wbc/search_ticks"', '"contact_alpha", "search"'):
            self.assertIn(needle, src, needle)
        self.assertIn("testTouchdownSearch();", (ROOT / "tests/whole_body_feedback_core_test.cpp").read_text())
        self.assertIn("search_longest_s", (ROOT / "tests/tuning/score_suite.py").read_text())

    def test_no_empty_lists_in_controller_configs(self):
        # rcl_yaml_param_parser cannot assign a type to `[]`; declaring such a key as double_array makes the
        # controller fail to load (controller_sequence.py exit 1, no locomotion) -- this cost WBC sweep 2 6/8 cases.
        def walk(node, path):
            if isinstance(node, dict):
                for k, v in node.items():
                    walk(v, path + [str(k)])
            elif isinstance(node, list):
                self.assertTrue(node, f"empty list at {'.'.join(path)}")
        for cfg in sorted(ROOT.glob("config/*/ros2_controllers.yaml")):
            walk(yaml.safe_load(cfg.read_text()), [cfg.parent.name])

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
        keys = set(schema_whole_body())
        for name in ("run_wbc_sweep.sh", "run_wbc_sweep2.sh", "run_wbc_sweep3.sh"):
            script = (ROOT / "tests/tuning" / name).read_text()
            for key in re.findall(r"\$WB\.(\w+)=", script):
                self.assertIn(key, keys, f"{name} sets unknown key control.wholeBody.{key}")
            self.assertIn("--no-default-guards", script)
        self.assertIn("meas_tau_hf", (ROOT / "tests/tuning/plot_tracking.py").read_text())

    def test_sweep2_waist_weights_match_joint_order(self):
        g1 = yaml.safe_load((ROOT / "config/g1/ros2_controllers.yaml").read_text())
        names = g1[CTRL]["ros__parameters"]["robot"]["jointNames"]
        script = (ROOT / "tests/tuning/run_wbc_sweep2.sh").read_text()
        weights = [float(v) for v in re.search(r"jointWeights=\[([^\]]+)\]", script).group(1).split(",")]
        self.assertEqual(len(weights), len(names))
        for w, n in zip(weights, names):
            self.assertEqual(w > 1.0, "waist" in n, f"{n}: weight {w}")
        self.assertIn("torso_link", script)
        self.assertEqual(g1[CTRL]["ros__parameters"]["ocs2"]["costs"]["torsoTiltCost"]["linkName"], "torso_link")

    def test_tracking_columns(self):
        src = (ROOT / "src/controller/controller.cpp").read_text()
        for name in ("active", "residual", "clips", "gain_alpha"):
            self.assertIn(f'"{name}"', src)


if __name__ == "__main__":
    unittest.main()
