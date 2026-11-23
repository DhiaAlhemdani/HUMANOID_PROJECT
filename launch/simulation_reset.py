#!/usr/bin/env python3
"""Coordinate a full MuJoCo world reset with ros2_control controller lifecycle."""

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger


RESET_SERVICE = "/legged_robot_mpc_controller/reset_simulation"
CONTROLLER_MANAGER = "/controller_manager"
SIMULATOR_RESET = "/mujoco_ros2_control/reset_simulation"
DEFAULT_MPC_CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"


class SimulationResetCoordinator(Node):
    def __init__(self):
        super().__init__("simulation_reset_coordinator")
        self._busy = False
        self.declare_parameter("mpc_controller_name", DEFAULT_MPC_CONTROLLER)
        self._mpc_controller_name = self.get_parameter("mpc_controller_name").value.strip("/")
        self._hold_after_reset_client = self.create_client(
            Trigger,
            f"/{self._mpc_controller_name}/hold_after_reset",
            callback_group=ReentrantCallbackGroup(),
        )
        self._callback_group = ReentrantCallbackGroup()
        self._list_client = self.create_client(
            ListControllers, f"{CONTROLLER_MANAGER}/list_controllers", callback_group=self._callback_group
        )
        self._switch_client = self.create_client(
            SwitchController, f"{CONTROLLER_MANAGER}/switch_controller", callback_group=self._callback_group
        )
        self._sim_reset_client = self.create_client(
            Trigger, SIMULATOR_RESET, callback_group=self._callback_group
        )
        self._service = self.create_service(
            Trigger, RESET_SERVICE, self._reset, callback_group=self._callback_group
        )
        self.get_logger().info(f"Coordinated reset ready on {RESET_SERVICE}")

    @staticmethod
    def _activation_order(controller_names):
        # Match launch/controller_sequence.py: state broadcasters and estimators
        # must be active before the MPC claims their state interfaces.
        def priority(name):
            if name in ("joint_state_broadcaster", "robot_state_broadcaster"):
                return 0
            if name == "humanoid_whole_body_inverse_dynamics_mpc_controller":
                return 2
            return 1

        return sorted(controller_names, key=priority)

    @staticmethod
    def _switch_request(activate=(), deactivate=()):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        if activate:
            request.strictness = getattr(SwitchController.Request, "BEST_EFFORT", 1)
            if hasattr(request, "activate_asap"):
                request.activate_asap = False
        else:
            request.strictness = SwitchController.Request.STRICT
        if hasattr(request, "timeout"):
            request.timeout = Duration(sec=30)
        return request

    async def _reset(self, _request, response):
        if self._busy:
            response.success = False
            response.message = "A coordinated reset is already in progress"
            return response
        clients = (
            ("controller_manager/list_controllers", self._list_client),
            ("controller_manager/switch_controller", self._switch_client),
            ("mujoco_ros2_control/reset_simulation", self._sim_reset_client),
        )
        missing = [name for name, client in clients if not client.service_is_ready()]
        if missing:
            response.success = False
            response.message = "Reset unavailable; service(s) not ready: " + ", ".join(missing)
            return response

        self._busy = True
        active_controllers = []
        controllers_deactivated = False
        reset_ok = False
        reset_message = ""
        try:
            listed = await self._list_client.call_async(ListControllers.Request())
            active_controllers = [c.name for c in listed.controller if c.state == "active"]
            self.get_logger().info(f"Active controllers before reset: {active_controllers or 'none'}")
            mpc_was_active = self._mpc_controller_name in active_controllers
            if mpc_was_active and not self._hold_after_reset_client.service_is_ready():
                response.success = False
                response.message = (
                    f"Cannot safely reset: /{self._mpc_controller_name}/hold_after_reset is not ready"
                )
                return response
            if active_controllers:
                request = self._switch_request(deactivate=active_controllers)
                stopped = await self._switch_client.call_async(request)
                if not stopped.ok:
                    response.success = False
                    response.message = "Controller manager refused to deactivate active controllers"
                    return response
                controllers_deactivated = True

            if mpc_was_active:
                held = await self._hold_after_reset_client.call_async(Trigger.Request())
                if not held.success:
                    response.success = False
                    response.message = f"MPC could not be set to passive after reset: {held.message}"
                    return response
                self.get_logger().info(
                    f"{self._mpc_controller_name} will reactivate in passive hold; stand-up will not auto-start"
                )

            result = await self._sim_reset_client.call_async(Trigger.Request())
            reset_ok = result.success
            reset_message = result.message if result.success else f"MuJoCo reset failed: {result.message}"
        except Exception as exc:
            reset_message = f"Coordinated reset failed: {exc}"
        finally:
            if controllers_deactivated:
                for name in self._activation_order(active_controllers):
                    try:
                        request = self._switch_request(activate=[name])
                        activated = await self._switch_client.call_async(request)
                        if not activated.ok:
                            reset_ok = False
                            reset_message += f"; failed to reactivate controller {name}"
                            self.get_logger().error(f"Failed to reactivate controller {name}")
                            break
                        self.get_logger().info(f"Reactivated controller {name}")
                    except Exception as exc:
                        reset_ok = False
                        reset_message += f"; controller {name} reactivation failed: {exc}"
                        break
            self._busy = False

        if reset_ok:
            response.success = True
            if active_controllers:
                response.message = "Simulation reset; reactivated controllers: " + ", ".join(
                    self._activation_order(active_controllers)
                )
                if self._mpc_controller_name in active_controllers:
                    response.message += (
                        f"; {self._mpc_controller_name} is passive and holding the reset pose. "
                        f"Call /{self._mpc_controller_name}/stand_up to stand."
                    )
            else:
                response.message = "Simulation reset; no controllers were active to restart"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = reset_message or "Reset failed"
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimulationResetCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
