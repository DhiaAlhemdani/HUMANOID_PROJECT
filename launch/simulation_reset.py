#!/usr/bin/env python3
"""Coordinate a full MuJoCo world reset with ros2_control controller lifecycle."""

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger


RESET_SERVICE = "/legged_robot_mpc_controller/reset_simulation"
CONTROLLER_MANAGER = "/controller_manager"
SIMULATOR_RESET = "/mujoco_ros2_control/reset_simulation"


class SimulationResetCoordinator(Node):
    def __init__(self):
        super().__init__("simulation_reset_coordinator")
        self._busy = False
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
            if active_controllers:
                request = SwitchController.Request()
                request.deactivate_controllers = active_controllers
                request.strictness = SwitchController.Request.STRICT
                stopped = await self._switch_client.call_async(request)
                if not stopped.ok:
                    response.success = False
                    response.message = "Controller manager refused to deactivate active controllers"
                    return response
                controllers_deactivated = True

            result = await self._sim_reset_client.call_async(Trigger.Request())
            reset_ok = result.success
            reset_message = result.message if result.success else f"MuJoCo reset failed: {result.message}"
        except Exception as exc:
            reset_message = f"Coordinated reset failed: {exc}"
        finally:
            if controllers_deactivated:
                request = SwitchController.Request()
                request.activate_controllers = active_controllers
                request.strictness = SwitchController.Request.STRICT
                try:
                    activated = await self._switch_client.call_async(request)
                    if not activated.ok:
                        reset_ok = False
                        reset_message += "; failed to reactivate controllers"
                except Exception as exc:
                    reset_ok = False
                    reset_message += f"; controller reactivation failed: {exc}"
            self._busy = False

        if reset_ok:
            response.success = True
            response.message = "Simulation and controller state reset; previously active controllers reactivated"
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
