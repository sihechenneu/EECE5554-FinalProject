"""ROS 2 port of Interbotix gripper interface for Universal Factory Xarm."""
import rclpy
from xarm_msgs.srv import GripperConfig, GripperMove, GripperState
from interbotix_ux_modules.core import InterbotixRobotUXCore


class InterbotixGripperUX(object):
    """Standalone module to control a Universal Factory Gripper."""

    def __init__(
        self,
        robot_model,
        robot_name=None,
        mode=0,
        wait_for_finish=True,
        ee_offset=None,
        init_node=True,
        joint_state_topic="arm/joint_states",
        pulse_vel=1500,
        pulse=850,
        gripper_type="gripper",
        node=None,
    ):
        self.ux = InterbotixRobotUXCore(
            robot_model,
            robot_name,
            mode,
            wait_for_finish,
            ee_offset,
            init_node,
            joint_state_topic,
            node=node,
        )
        if gripper_type == "gripper":
            self.gripper = InterbotixGripperUXInterface(self.ux, pulse_vel, pulse)


class InterbotixGripperUXInterface(object):
    """Gripper interface using InterbotixRobotUXCore."""

    def __init__(self, core, pulse_vel=1500, pulse=850):
        self.core = core
        node = self.core._node
        self._srv_gripper_move = node.create_client(GripperMove, "gripper_move")
        self._srv_gripper_config = node.create_client(GripperConfig, "gripper_config")
        self._srv_gripper_state = node.create_client(GripperState, "gripper_state")
        for client in [self._srv_gripper_move, self._srv_gripper_config, self._srv_gripper_state]:
            while not client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
                node.get_logger().info("Waiting for service %s..." % client.srv_name)
        self.config(pulse_vel)
        self.move(pulse)
        node.get_logger().info("Initializing InterbotixGripperUXInterface...")
        node.get_logger().info("Gripper Pulse Vel: %s  Gripper Pulse: %d" % (pulse_vel, pulse))

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.core._node, future, timeout_sec=5.0)
        return future.result()

    def move(self, pulse, delay=1.0):
        req = GripperMove.Request()
        req.pulse_pos = float(pulse)
        resp = self._call(self._srv_gripper_move, req)
        if resp.ret != 0:
            self.core._node.get_logger().info(resp.message)
        self.core._node.get_clock().sleep_for(rclpy.duration.Duration(seconds=delay))
        return resp.ret

    def config(self, pulse_vel):
        req = GripperConfig.Request()
        req.pulse_vel = float(pulse_vel)
        resp = self._call(self._srv_gripper_config, req)
        self.core._node.get_logger().info(resp.message)
        return resp.ret

    def get_state(self):
        req = GripperState.Request()
        resp = self._call(self._srv_gripper_state, req)
        if resp.err_code != 0:
            self.core._node.get_logger().info("Error Num: %d" % resp.err_code)
        state = (850 - resp.curr_pos) / 1000.0
        return state

    def open(self, delay=1.0):
        return self.move(850, delay)

    def close(self, delay=1.0):
        return self.move(0, delay)
