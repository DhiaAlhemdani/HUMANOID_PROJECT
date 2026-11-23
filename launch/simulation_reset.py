#!/usr/bin/env python3
"""Reset MuJoCo and ros2_control as one coordinated operation."""

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger


RESET_SERVICE = "/legged_robot_mpc_controller/reset_simulation"
CONTROLLER_MANAGER = "/controller_manager"
SIMULATOR_RESET = "/mujoco_ros2_control/reset_simulation"
MPC_CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"


class SimulationResetCoordinator(Node):
    def __init__(self):
        super().__init__("simulation_reset_coordinator")
        self._busy = False
        self._group = ReentrantCallbackGroup()
        self._list_client = self.create_client(ListControllers, f"{CONTROLLER_MANAGER}/list_controllers", callback_group=self._group)
        self._switch_client = self.create_client(SwitchController, f"{CONTROLLER_MANAGER}/switch_controller", callback_group=self._group)
        self._sim_reset_client = self.create_client(Trigger, SIMULATOR_RESET, callback_group=self._group)
        self.create_service(Trigger, RESET_SERVICE, self._reset, callback_group=self._group)
        self.get_logger().info(f"Coordinated reset ready on {RESET_SERVICE}")

    @staticmethod
    def _switch_request(activate=(), deactivate=()):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.timeout = Duration(sec=30)
        if activate:
            request.activate_asap = False
        return request

    @staticmethod
    def _activation_order(names):
        def priority(name):
            if name in ("joint_state_broadcaster", "robot_state_broadcaster"):
                return 0
            if name == MPC_CONTROLLER:
                return 2
            return 1
        return sorted(names, key=priority)

    async def _reset(self, _request, response):
        if self._busy:
            response.success = False
            response.message = "A reset is already in progress"
            return response
        clients = (
            ("list_controllers", self._list_client),
            ("switch_controller", self._switch_client),
            ("MuJoCo reset", self._sim_reset_client),
        )
        unavailable = [name for name, client in clients if not client.service_is_ready()]
        if unavailable:
            response.success = False
            response.message = "Reset services unavailable: " + ", ".join(unavailable)
            return response

        self._busy = True
        active = []
        deactivated = False
        reset_ok = False
        message = "Reset failed"
        try:
            listed = await self._list_client.call_async(ListControllers.Request())
            active = [controller.name for controller in listed.controller if controller.state == "active"]
            self.get_logger().info(f"Active controllers before reset: {active or 'none'}")
            if active:
                stopped = await self._switch_client.call_async(self._switch_request(deactivate=active))
                if not stopped.ok:
                    message = "Controller manager refused to deactivate active controllers"
                    return self._fail(response, message)
                deactivated = True

            result = await self._sim_reset_client.call_async(Trigger.Request())
            reset_ok = result.success
            message = result.message if result.success else f"MuJoCo built-in reset failed: {result.message}"
        except Exception as exc:
            message = f"Coordinated reset failed: {exc}"
        finally:
            if deactivated:
                for name in self._activation_order(active):
                    try:
                        activated = await self._switch_client.call_async(self._switch_request(activate=[name]))
                        if not activated.ok:
                            reset_ok = False
                            message += f"; could not reactivate {name}"
                            break
                        self.get_logger().info(f"Reactivated controller {name}")
                    except Exception as exc:
                        reset_ok = False
                        message += f"; reactivation of {name} failed: {exc}"
                        break
            self._busy = False

        response.success = reset_ok
        response.message = message if not reset_ok else "Simulation and active controllers reset successfully"
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response

    @staticmethod
    def _fail(response, message):
        response.success = False
        response.message = message
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
