#!/usr/bin/env python3
"""Static wiring checks for the double-support load-share cost (D2/D3 structural lever).

These do not execute the C++ OCP; they pin the parameter plumbing, the off-by-default
registration gate, and the cost's double-support / wrench-layout assumptions.
"""

import re
import unittest

import yaml

from wb_id_parameter_contract_test import NAME, ROOT, UniqueLoader

SCHEMA = yaml.load((ROOT / "src/controller/parameters.yaml").read_text(), Loader=UniqueLoader)[NAME]
PARAMS = yaml.load((ROOT / "config/g1/ros2_controllers.yaml").read_text(), Loader=UniqueLoader)[NAME]["ros__parameters"]
BUILDER = (ROOT / "src/controller/config_builder.cpp").read_text()
INTERFACE_H = (ROOT / "include/legged_robot_mpc_controller/mpc/wb_id/WbIdMpcInterface.h").read_text()
INTERFACE_CPP = (ROOT / "src/mpc/wb_id/WbIdMpcInterface.cpp").read_text()
COST_H = (ROOT / "include/legged_robot_mpc_controller/mpc/wb_id/cost/WbIdContactLoadShareCost.h").read_text()
COST_CPP = (ROOT / "src/mpc/wb_id/cost/WbIdContactLoadShareCost.cpp").read_text()
CMAKE = (ROOT / "CMakeLists.txt").read_text()


def block(source, marker):
    start = source.index("{", source.index(marker))
    depth = 1
    for end in range(start + 1, len(source)):
        depth += (source[end] == "{") - (source[end] == "}")
        if depth == 0:
            return source[start + 1 : end]
    raise AssertionError(f"Unclosed block: {marker}")


class LoadShareContractTest(unittest.TestCase):
    def test_schema_entry_defaults_off(self):
        entry = SCHEMA["ocs2"]["costs"]["contactLoadShareWeight"]
        self.assertEqual(entry["type"], "double")
        self.assertEqual(entry["default_value"], 0.0)
        self.assertIn("gt_eq", entry["validation"])

    def test_shipped_config_shapes_the_lift_off_end_only(self):
        # G1 ships the lift-off end window (docs/results/human_gait_template.md run 14): the whole-DS
        # smoothstep (both windows 0) fell at 0.8 m/s and moved the jerk onto the touchdown.
        costs = PARAMS["ocs2"]["costs"]
        self.assertEqual(costs["contactLoadShareWeight"], 0.1)
        self.assertEqual(costs["contactLoadShareLiftOffWindow"], 0.06)
        self.assertEqual(costs["contactLoadShareTouchDownWindow"], 0.0)
        for key in ("contactLoadShareLiftOffWindow", "contactLoadShareTouchDownWindow"):
            self.assertEqual(SCHEMA["ocs2"]["costs"][key]["default_value"], 0.0)
            self.assertIn(f"config.{key} = p.costs.{key};", BUILDER)

    def test_builder_maps_weight(self):
        self.assertIn("config.contactLoadShareWeight = p.costs.contactLoadShareWeight;", BUILDER)

    def test_interface_config_field_defaults_off(self):
        self.assertRegex(INTERFACE_H, r"scalar_t contactLoadShareWeight = 0\.0;")

    def test_registration_is_gated_on_positive_weight(self):
        # The cost must be added only inside an explicit > 0 guard (shipped behavior is inert).
        self.assertIn('costPtr->add("contactLoadShareCost"', INTERFACE_CPP)
        guard = block(INTERFACE_CPP, "if (config_.contactLoadShareWeight > 0.0)")
        self.assertIn('add("contactLoadShareCost"', guard)
        self.assertNotIn("contactLoadShareWeight > 0.0", COST_CPP)  # ctor itself rejects 0

    def test_cost_gates_to_double_support(self):
        self.assertIn("getContactFlags", COST_CPP)
        self.assertRegex(COST_CPP, r"return flags\[0\] && flags\[1\];")

    def test_cost_reads_vertical_wrench_components(self):
        # u = [W_l(6), W_r(6), ...], W = [f; m] world-aligned -> f_z at 6*i + 2.
        self.assertIn("input(2)", COST_CPP)
        self.assertIn("input(8)", COST_CPP)

    def test_parameters_vector_layout(self):
        # [sqrt(weight), phi_left, phi_right]
        self.assertIn("vector_t params(3);", COST_CPP)
        self.assertIn("params(1) = phi0;", COST_CPP)
        self.assertIn("params(2) = phi1;", COST_CPP)

    def test_residual_is_total_invariant(self):
        guard = block(COST_H, "inline void splitResidual")
        self.assertIn("wz0 + wz1", guard)
        self.assertIn("wz0 - phi0 * total", guard)
        self.assertIn("wz1 - phi1 * total", guard)

    def test_free_function_extraction_markers_present(self):
        self.assertEqual(COST_H.count("[load_share_free_functions_begin]"), 1)
        self.assertEqual(COST_H.count("[load_share_free_functions_end]"), 1)

    def test_cmake_globs_wb_id_sources(self):
        # The new cost .cpp is picked up by the recursive glob, not an explicit list.
        self.assertRegex(CMAKE, r"GLOB_RECURSE\s+HUMANOID_WB_ID_CORE_SOURCES\s+CONFIGURE_DEPENDS\s+src/mpc/wb_id/\*\.cpp")

    def test_smoothstep_is_c1(self):
        # Zero slope at both ends of the transfer window (no target-rate jump at handover).
        guard = block(COST_H, "inline double smoothstep01")
        self.assertIn("x * x * (3.0 - 2.0 * x)", guard)


if __name__ == "__main__":
    unittest.main()
