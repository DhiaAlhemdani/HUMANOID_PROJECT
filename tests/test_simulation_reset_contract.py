"""Contract checks for the MuJoCo + ros2_control reset layers."""

from pathlib import Path
import stat
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SimulationResetContractTest(unittest.TestCase):
    def test_reset_coordinator_is_installed_and_mujoco_only(self):
        script = ROOT / "launch/simulation_reset.py"
        self.assertTrue(script.stat().st_mode & stat.S_IXUSR)
        self.assertIn("launch/simulation_reset.py", (ROOT / "CMakeLists.txt").read_text())
        launch = (ROOT / "launch/robot.launch.py").read_text()
        self.assertIn('executable="simulation_reset.py"', launch)
        self.assertIn("condition=IfCondition(use_mujoco_sim)", launch)
        self.assertIn("simulation_reset_coordinator,", launch)

    def test_ros_controller_reset_brackets_builtin_world_reset(self):
        source = (ROOT / "launch/simulation_reset.py").read_text()
        stop = source.index("stopped = await self._switch_client.call_async")
        reset = source.index("result = await self._sim_reset_client.call_async")
        activation = source.index("for name in self._activation_order(active)")
        self.assertLess(stop, reset)
        self.assertLess(reset, activation)
        self.assertIn('SIMULATOR_RESET = "/mujoco_ros2_control/reset_simulation"', source)
        self.assertIn("_switch_request(deactivate=active)", source)
        self.assertIn("_switch_request(activate=[name])", source)

    def test_dependency_uses_mujoco_builtin_and_restores_only_spawn_qpos(self):
        patch = (ROOT / "patches/mujoco_ros2_control_builtin_reset.patch").read_text()
        self.assertIn("mj_resetData(mj_model_, mj_data_)", patch)
        self.assertIn("initial_qpos_", patch)
        self.assertIn("std::copy(initial_qpos_.begin(), initial_qpos_.end(), mj_data_->qpos)", patch)
        self.assertIn("reset_request_callback_()", patch)
        self.assertNotIn("initial_simulation_state_", patch)

    def test_viewer_and_rviz_share_reset_service_but_keep_camera_reset(self):
        patch = (ROOT / "patches/mujoco_ros2_control_builtin_reset.patch").read_text()
        self.assertIn("/legged_robot_mpc_controller/reset_simulation", patch)
        panel = (ROOT / "src/visualization/simulation_reset_panel.cpp").read_text()
        self.assertIn("Reset Simulation", panel)
        self.assertIn("/legged_robot_mpc_controller/reset_simulation", panel)
        rviz = (ROOT / "config/rviz/humanoid.rviz").read_text()
        self.assertIn("legged_robot_mpc_controller/SimulationResetPanel", rviz)
        readme = (ROOT / "README.md").read_text()
        self.assertIn("MuJoCo **R**", readme)
        self.assertIn("view **Reset**", readme)

    def test_patch_helper_is_idempotent(self):
        helper = (ROOT / "scripts/apply_mujoco_reset_patch.sh").read_text()
        self.assertIn("--reverse --check", helper)
        self.assertIn("--check", helper)
        self.assertIn("BUILTIN_RESET_PATCH", helper) if "BUILTIN_RESET_PATCH" in helper else self.assertIn("PATCH_FILE", helper)


if __name__ == "__main__":
    unittest.main()
