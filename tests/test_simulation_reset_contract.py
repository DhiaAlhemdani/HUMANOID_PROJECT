"""Static contract checks for the coordinated MuJoCo/RViz reset path."""

from pathlib import Path
import stat
import unittest


ROOT = Path(__file__).resolve().parents[1]


class SimulationResetContractTest(unittest.TestCase):
    def test_reset_coordinator_is_installed_as_an_executable(self):
        script = ROOT / "launch/simulation_reset.py"
        self.assertTrue(script.stat().st_mode & stat.S_IXUSR)
        self.assertIn("launch/simulation_reset.py", (ROOT / "CMakeLists.txt").read_text())

    def test_launch_starts_reset_coordinator_only_for_mujoco(self):
        launch = (ROOT / "launch/robot.launch.py").read_text()
        block = launch[launch.index("simulation_reset_coordinator = Node("):]
        block = block[:block.index("rviz_node = Node(")]
        self.assertIn('executable="simulation_reset.py"', block)
        self.assertIn("condition=IfCondition(use_mujoco_sim)", block)
        self.assertIn("simulation_reset_coordinator,", launch)

    def test_rviz_default_layout_exposes_shared_reset_service(self):
        rviz = (ROOT / "config/rviz/humanoid.rviz").read_text()
        panel = (ROOT / "src/visualization/simulation_reset_panel.cpp").read_text()
        coordinator = (ROOT / "launch/simulation_reset.py").read_text()
        self.assertIn("legged_robot_mpc_controller/SimulationResetPanel", rviz)
        self.assertIn("Reset Simulation", panel)
        self.assertIn("/legged_robot_mpc_controller/reset_simulation", panel)
        self.assertIn('RESET_SERVICE = "/legged_robot_mpc_controller/reset_simulation"', coordinator)

    def test_coordinator_deactivates_before_world_reset_and_reactivates(self):
        source = (ROOT / "launch/simulation_reset.py").read_text()
        self.assertIn("_switch_request(deactivate=active_controllers)", source)
        self.assertIn("for name in self._activation_order(active_controllers)", source)
        self.assertIn("_switch_request(activate=[name])", source)
        self.assertIn('name == "humanoid_whole_body_inverse_dynamics_mpc_controller"', source)
        self.assertLess(
            source.index("stopped = await self._switch_client.call_async(request)"),
            source.index("result = await self._sim_reset_client.call_async(Trigger.Request())"),
        )
        self.assertIn("SIMULATOR_RESET = \"/mujoco_ros2_control/reset_simulation\"", source)

    def test_dependency_patch_and_camera_controls_are_documented(self):
        patch = ROOT / "patches/mujoco_ros2_control_reset.patch"
        self.assertTrue(patch.is_file())
        self.assertGreater(patch.stat().st_size, 1000)
        self.assertIn("pre-reset time", patch.read_text())
        followup = ROOT / "patches/mujoco_ros2_control_reset_followup.patch"
        self.assertTrue(followup.is_file())
        self.assertIn("last_cam_update = simstart", followup.read_text())
        script = (ROOT / "scripts/apply_mujoco_reset_patch.sh").read_text()
        self.assertIn("FOLLOWUP_PATCH", script)
        self.assertIn("Upgraded coordinated reset support", script)
        self.assertIn("git -C \"${DEPENDENCY}\" apply --ignore-whitespace --check", script)
        readme = (ROOT / "README.md").read_text()
        for control in ("Backspace", "Reset Simulation", "MuJoCo viewer's **R** key", "RViz's standard **Reset**"):
            self.assertIn(control, readme)
        update_section = readme.split("### Updating an existing local workspace from this repository", 1)[1]
        for command in ("fetch arena", "pull --ff-only arena", "colcon build --packages-select mujoco_ros2_control",
                        "colcon build --packages-up-to legged_robot_mpc_controller", "source install/setup.bash"):
            self.assertIn(command, update_section)


if __name__ == "__main__":
    unittest.main()
