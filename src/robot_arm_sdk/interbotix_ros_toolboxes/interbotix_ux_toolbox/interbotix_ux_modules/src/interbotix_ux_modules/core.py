"""ROS 2 port of InterbotixRobotUXCore - control Universal Factory Xarm via xarm_api services."""
import copy
import threading
import rclpy
from rclpy.node import Node
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import (
    ClearErr,
    GetErr,
    Move,
    SetAxis,
    SetInt16,
    SetLoad,
    TCPOffset,
)
from sensor_msgs.msg import JointState


class InterbotixRobotUXCore(object):
    """Standalone module to control a Universal Factory Xarm via ROS 2."""

    def __init__(
        self,
        robot_model,
        robot_name=None,
        mode=0,
        wait_for_finish=True,
        ee_offset=None,
        init_node=True,
        joint_state_topic="arm/joint_states",
        node=None,
    ):
        self.joint_states = None
        self.xarm_states = None
        self.ee_offset = ee_offset
        self.xs_mutex = threading.Lock()
        self.js_mutex = threading.Lock()
        self.robot_model = robot_model
        self.robot_name = robot_name if robot_name is not None else robot_model

        if node is not None:
            self._node = node
            self._owns_node = False
        elif init_node:
            rclpy.init()
            self._node = rclpy.create_node(
                self.robot_name + "_robot_manipulation",
                namespace=self.robot_name,
            )
            self._owns_node = True
        else:
            raise ValueError("Either node or init_node=True must be provided")

        self._node.declare_parameter("DOF", 6)
        self._node.declare_parameter("joint_names", [])
        self.dof = self._node.get_parameter("DOF").value
        self.joint_names = list(self._node.get_parameter("joint_names").value)
        if not self.joint_names:
            self.joint_names = [f"joint{i+1}" for i in range(self.dof)]

        # Service clients (topic names relative to node namespace)
        self._srv_motion_ctrl = self._node.create_client(SetAxis, "motion_ctrl")
        self._srv_get_err = self._node.create_client(GetErr, "get_err")
        self._srv_clear_err = self._node.create_client(ClearErr, "clear_err")
        self._srv_set_mode = self._node.create_client(SetInt16, "set_mode")
        self._srv_set_state = self._node.create_client(SetInt16, "set_state")
        self._srv_set_load = self._node.create_client(SetLoad, "set_load")
        self._srv_set_tcp = self._node.create_client(TCPOffset, "set_tcp_offset")
        self._srv_go_home = self._node.create_client(Move, "go_home")
        self._srv_move_line = self._node.create_client(Move, "move_line")
        self._srv_move_lineb = self._node.create_client(Move, "move_lineb")
        self._srv_move_joint = self._node.create_client(Move, "move_joint")
        self._srv_move_servoj = self._node.create_client(Move, "move_servoj")
        self._srv_move_servo_cart = self._node.create_client(Move, "move_servo_cart")

        for client in [
            self._srv_motion_ctrl,
            self._srv_get_err,
            self._srv_clear_err,
            self._srv_set_mode,
            self._srv_set_state,
            self._srv_set_load,
            self._srv_set_tcp,
            self._srv_go_home,
            self._srv_move_line,
            self._srv_move_lineb,
            self._srv_move_joint,
            self._srv_move_servoj,
            self._srv_move_servo_cart,
        ]:
            while not client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
                self._node.get_logger().info("Waiting for service %s..." % client.srv_name)

        self._sub_joint_states = self._node.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_cb,
            10,
        )
        self._sub_xarm_states = self._node.create_subscription(
            RobotMsg,
            "xarm_states",
            self.xarm_state_cb,
            10,
        )

        self._node.get_logger().info("Initializing InterbotixRobotUXCore...")
        self._node.get_logger().info("Robot Name: %s  Robot Model: %s" % (self.robot_name, robot_model))

        while self.joint_states is None and self.xarm_states is None and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        self.js_index_map = dict(zip(self.joint_states.name, range(len(self.joint_states.name))))
        self.mode = mode
        self._node.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))
        self.robot_motion_enable(8, True)
        if self.ee_offset is not None:
            ee_off = self.ee_offset[:]
            ee_off[0] *= 1000
            ee_off[1] *= 1000
            ee_off[2] *= 1000
            self.robot_set_tcp_offset(ee_off)
        self.robot_smart_mode_reset(self.mode)

    def _call_srv(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        return future.result()

    def robot_motion_enable(self, id=8, enable=True):
        req = SetAxis.Request()
        req.id = int(id)
        req.data = 1 if enable else 0
        resp = self._call_srv(self._srv_motion_ctrl, req)
        self._node.get_logger().info(resp.message)
        return resp.ret

    def robot_get_error(self):
        req = GetErr.Request()
        resp = self._call_srv(self._srv_get_err, req)
        self._node.get_logger().info(resp.message)
        return resp.err

    def robot_clear_error(self):
        req = ClearErr.Request()
        resp = self._call_srv(self._srv_clear_err, req)
        self._node.get_logger().info(resp.message)
        return resp.ret

    def robot_set_mode(self, mode=0):
        req = SetInt16.Request()
        req.data = int(mode)
        resp = self._call_srv(self._srv_set_mode, req)
        if resp.ret != 0:
            self.mode = 0
        else:
            self.mode = mode
        self._node.get_logger().info(resp.message)
        return resp.ret

    def robot_set_state(self, state=0):
        req = SetInt16.Request()
        req.data = int(state)
        resp = self._call_srv(self._srv_set_state, req)
        self._node.get_logger().info(resp.message)
        return resp.ret

    def robot_set_load(self, mass, xc=0, yc=0, zc=0):
        req = SetLoad.Request()
        req.mass = float(mass)
        req.xc = float(xc)
        req.yc = float(yc)
        req.zc = float(zc)
        resp = self._call_srv(self._srv_set_load, req)
        self._node.get_logger().info(resp.message)
        return resp.ret

    def robot_set_tcp_offset(self, ee_offset):
        req = TCPOffset.Request()
        req.x = float(ee_offset[0])
        req.y = float(ee_offset[1])
        req.z = float(ee_offset[2])
        req.roll = float(ee_offset[3])
        req.pitch = float(ee_offset[4])
        req.yaw = float(ee_offset[5])
        resp = self._call_srv(self._srv_set_tcp, req)
        self.robot_set_state(0)
        self._node.get_logger().info(resp.message)
        return resp.ret

    def robot_go_home(self, vel=0.35, accel=7):
        req = Move.Request()
        req.mvvelo = float(vel)
        req.mvacc = float(accel)
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.pose = []
        resp = self._call_srv(self._srv_go_home, req)
        self._node.get_logger().info(resp.message)
        return resp.ret

    def robot_move_line(self, pose, vel=200, accel=2000):
        req = Move.Request()
        req.pose = [float(p) for p in pose]
        req.mvvelo = float(vel)
        req.mvacc = float(accel)
        req.mvtime = 0.0
        req.mvradii = 0.0
        resp = self._call_srv(self._srv_move_line, req)
        if resp.ret != 0:
            self._node.get_logger().info(resp.message)
        return resp.ret

    def robot_move_lineb(self, num_points, pose_list, vel=200, accel=2000, radii=0):
        for point in range(num_points):
            req = Move.Request()
            req.pose = [float(p) for p in pose_list[point]]
            req.mvvelo = float(vel)
            req.mvacc = float(accel)
            req.mvtime = 0.0
            req.mvradii = float(radii)
            resp = self._call_srv(self._srv_move_lineb, req)
            if resp.ret != 0:
                self._node.get_logger().info(resp.message)
                return resp.ret
        return 0

    def robot_move_joint(self, cmd, vel=1.0, accel=5.0):
        req = Move.Request()
        req.pose = [float(c) for c in cmd]
        req.mvvelo = float(vel)
        req.mvacc = float(accel)
        req.mvtime = 0.0
        req.mvradii = 0.0
        resp = self._call_srv(self._srv_move_joint, req)
        if resp.ret != 0:
            self._node.get_logger().info(resp.message)
        return resp.ret

    def robot_move_servoj(self, cmd):
        req = Move.Request()
        req.pose = [float(c) for c in cmd]
        req.mvvelo = 0.0
        req.mvacc = 0.0
        req.mvtime = 0.0
        req.mvradii = 0.0
        resp = self._call_srv(self._srv_move_servoj, req)
        if resp.ret != 0:
            self._node.get_logger().info(resp.message)
        return resp.ret

    def robot_move_servo_cart(self, cmd):
        req = Move.Request()
        req.pose = [float(c) for c in cmd]
        req.mvvelo = 0.0
        req.mvacc = 0.0
        req.mvtime = 0.0
        req.mvradii = 0.0
        resp = self._call_srv(self._srv_move_servo_cart, req)
        if resp.ret != 0:
            self._node.get_logger().info(resp.message)
        return resp.ret

    def robot_smart_mode_reset(self, mode=0):
        with self.xs_mutex:
            current_mode = self.xarm_states.mode
        while mode != current_mode:
            ret = self.robot_set_mode(mode)
            while ret != 0:
                self.robot_clear_error()
                ret = self.robot_set_mode(mode)
            self.robot_set_state(0)
            self._node.get_clock().sleep_for(rclpy.duration.Duration(nanoseconds=500000000))
            with self.xs_mutex:
                current_mode = self.xarm_states.mode

    def robot_get_single_joint_state(self, name):
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        joint_index = joint_states.name.index(name)
        return {
            "position": joint_states.position[joint_index],
            "velocity": joint_states.velocity[joint_index] if joint_states.velocity else 0.0,
            "effort": joint_states.effort[joint_index] if joint_states.effort else 0.0,
        }

    def robot_get_joint_states(self):
        with self.js_mutex:
            return copy.deepcopy(self.joint_states)

    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg

    def xarm_state_cb(self, msg):
        with self.xs_mutex:
            self.xarm_states = msg
