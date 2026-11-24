#!/usr/bin/env python3
"""Static WB-ID solve-handover command-blend contract; no ROS build or simulator required."""

from pathlib import Path
import unittest

import yaml

from wb_id_parameter_contract_test import NAME, UniqueLoader, check_keys


ROOT = Path(__file__).resolve().parents[1]
SCHEMA = yaml.load((ROOT / "src/controller/parameters.yaml").read_text(), Loader=UniqueLoader)[NAME]
CONTROLLER = (ROOT / "src/controller/controller.cpp").read_text()
HEADER = (ROOT / "include/legged_robot_mpc_controller/controller/controller.hpp").read_text()


def body(name):
    return CONTROLLER.split(f"HumanoidWholeBodyInverseDynamicsMpcController::{name}(", 1)[1].split("\n}\n", 1)[0]


class PolicySwapBlendContractTest(unittest.TestCase):
    def test_parameter_is_declared_validated_and_mapped(self):
        entry = SCHEMA["control"]["solveBlendSeconds"]
        self.assertEqual(entry["type"], "double")
        self.assertEqual(entry["default_value"], 0.005)
        self.assertEqual(entry["validation"], {"gt_eq": [0.0]})
        self.assertIn("parameters_.control.solveBlendSeconds", CONTROLLER)
        self.assertEqual(CONTROLLER.count("parameters_.control.solveBlendSeconds"), 1)

    def test_robot_configs_set_a_blend_shorter_than_a_solve_period(self):
        # Ramp must finish before the next solve, otherwise consecutive ramps overlap
        # and the command only lags: 5 ms <= half the 80-100 Hz G1/H2 solve period,
        # 2 ms at Upkie's 200 Hz.
        limits = {"g1": 0.005, "h2": 0.005, "upkie": 0.002}
        for robot, expected in limits.items():
            config = yaml.load(
                (ROOT / f"config/{robot}/ros2_controllers.yaml").read_text(), Loader=UniqueLoader)
            check_keys(config[NAME]["ros__parameters"], SCHEMA)
            value = config[NAME]["ros__parameters"]["control"]["solveBlendSeconds"]
            with self.subTest(robot=robot):
                self.assertEqual(value, expected)
                self.assertGreater(value, 0.0)
                self.assertLessEqual(value, 0.005)

    def test_blend_runs_between_command_computation_and_write(self):
        update = body("update_and_write_commands")
        compute = update.index("command = run_bring_up(time, observation)")
        blend = update.index("command = apply_policy_swap_blend(std::move(command), observation.time, policy_updated)")
        write = update.index("write_joint_action_command(command.policy_position, command.policy_velocity, command.feedforward)")
        publish = update.index("publish_tracking(observation, command)")
        self.assertLess(compute, blend)
        self.assertLess(blend, write)
        self.assertLess(write, publish)  # tracking reports the blended, written command

    def test_blend_starts_on_a_swapped_policy_from_the_last_written_command(self):
        blend = body("apply_policy_swap_blend")
        self.assertIn("policy_updated && has_last_written_command_ && same_size(command, last_written_command_)", blend)
        self.assertIn("last_policy_blend_from_ = last_written_command_;", blend)
        self.assertIn("policy_blend_start_time_ = now;", blend)
        # Smoothstep ramp on position, velocity and feedforward, torque limits re-applied.
        self.assertIn("a * a * (3.0 - 2.0 * a)", blend)
        self.assertIn("(1.0 - s) * last_policy_blend_from_.feedforward + s * command.feedforward", blend)
        self.assertIn("clamp_torque(command.feedforward);", blend)
        # The written record is the blended output, so a mid-ramp re-swap stays continuous.
        self.assertIn("last_written_command_ = command;", blend)
        # Backward sim clock re-anchors instead of freezing the ramp.
        self.assertIn("now < policy_blend_start_time_", blend)

    def test_activation_clears_blend_state_and_declaration_exists(self):
        activate = body("on_activate")
        self.assertIn("has_last_written_command_ = false", activate)
        self.assertIn("policy_blend_start_time_ = -1.0", activate)
        self.assertIn(
            "JointActionCommand apply_policy_swap_blend(JointActionCommand command, double now, bool policy_updated);",
            HEADER)

    def test_zero_duration_disables_the_ramp(self):
        blend = body("apply_policy_swap_blend")
        self.assertIn("if (duration > 0.0)", blend)


if __name__ == "__main__":
    unittest.main()
