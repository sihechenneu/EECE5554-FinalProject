import copy
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import threading
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand, JointTrajectoryCommand
from interbotix_xs_msgs.srv import OperatingModes, MotorGains, RegisterValues, RobotInfo, TorqueEnable, Reboot
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class InterbotixRobotXSCore(object):
    """Class that interfaces with the xs_sdk node ROS interfaces."""

    def __init__(self, robot_model, robot_name=None, init_node=True, joint_state_topic="joint_states", node=None):
        self.joint_states = None
        self.js_mutex = threading.Lock()
        self.robot_name = robot_name
        if self.robot_name is None:
            self.robot_name = robot_model
        self._node = None
        self._executor = None
        if init_node:
            rclpy.init()
            self._node = Node(self.robot_name + "_robot_manipulation")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
        elif node is not None:
            self._node = node
        else:
            raise ValueError("When init_node=False, node must be provided.")

        base = "/" + self.robot_name + "/"
        self._srv_set_op_modes = self._node.create_client(OperatingModes, base + "set_operating_modes")
        self._srv_set_pids = self._node.create_client(MotorGains, base + "set_motor_pid_gains")
        self._srv_set_reg = self._node.create_client(RegisterValues, base + "set_motor_registers")
        self._srv_get_reg = self._node.create_client(RegisterValues, base + "get_motor_registers")
        self._srv_get_info = self._node.create_client(RobotInfo, base + "get_robot_info")
        self._srv_torque = self._node.create_client(TorqueEnable, base + "torque_enable")
        self._srv_reboot = self._node.create_client(Reboot, base + "reboot_motors")
        self.pub_group = self._node.create_publisher(JointGroupCommand, base + "commands/joint_group", 1)
        self.pub_single = self._node.create_publisher(JointSingleCommand, base + "commands/joint_single", 1)
        self.pub_traj = self._node.create_publisher(JointTrajectoryCommand, base + "commands/joint_trajectory", 1)
        self._node.create_subscription(JointState, base + joint_state_topic, self._joint_state_cb, 1)

        if not self._srv_set_op_modes.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error(
                "The robot '%s' is not discoverable. "
                "Did you enter the correct robot_name parameter? "
                "Is the xs_sdk node running? Quitting..." % self.robot_name
            )
            sys.exit(1)

        while self.joint_states is None and rclpy.ok():
            if self._executor is not None:
                self._executor.spin_once(timeout_sec=0.1)
            else:
                rclpy.spin_once(self._node, timeout_sec=0.1)

        self.js_index_map = dict(zip(self.joint_states.name, range(len(self.joint_states.name))))
        import time
        time.sleep(0.5)
        print("Robot Name: %s\nRobot Model: %s" % (self.robot_name, robot_model))
        print("Initialized InterbotixRobotXSCore!\n")

    @property
    def node(self):
        return self._node

    def _call_srv(self, client, request, timeout_sec=5.0):
        future = client.call_async(request)
        if self._executor is not None:
            self._executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        else:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        return future.result()

    def robot_set_operating_modes(self, cmd_type, name, mode, profile_type="velocity", profile_velocity=0, profile_acceleration=0):
        req = OperatingModes.Request()
        req.cmd_type = cmd_type
        req.name = name
        req.mode = mode
        req.profile_type = profile_type
        req.profile_velocity = profile_velocity
        req.profile_acceleration = profile_acceleration
        self._call_srv(self._srv_set_op_modes, req)

    def robot_set_motor_pid_gains(self, cmd_type, name, kp_pos, ki_pos=0, kd_pos=0, k1=0, k2=0, kp_vel=100, ki_vel=1920):
        req = MotorGains.Request()
        req.cmd_type = cmd_type
        req.name = name
        req.kp_pos = kp_pos
        req.ki_pos = ki_pos
        req.kd_pos = kd_pos
        req.k1 = k1
        req.k2 = k2
        req.kp_vel = kp_vel
        req.ki_vel = ki_vel
        self._call_srv(self._srv_set_pids, req)

    def robot_set_motor_registers(self, cmd_type, name, reg, value):
        req = RegisterValues.Request()
        req.cmd_type = cmd_type
        req.name = name
        req.reg = reg
        req.value = value
        self._call_srv(self._srv_set_reg, req)

    def robot_get_motor_registers(self, cmd_type, name, reg):
        req = RegisterValues.Request()
        req.cmd_type = cmd_type
        req.name = name
        req.reg = reg
        return self._call_srv(self._srv_get_reg, req)

    def robot_get_robot_info(self, cmd_type, name):
        req = RobotInfo.Request()
        req.cmd_type = cmd_type
        req.name = name
        return self._call_srv(self._srv_get_info, req)

    def srv_get_info(self, cmd_type, name):
        return self.robot_get_robot_info(cmd_type, name)

    def srv_set_reg(self, cmd_type, name, reg, value):
        self.robot_set_motor_registers(cmd_type, name, reg, value)

    def srv_set_op_modes(self, cmd_type, name, mode, profile_type="velocity", profile_velocity=0, profile_acceleration=0):
        self.robot_set_operating_modes(cmd_type, name, mode, profile_type, profile_velocity, profile_acceleration)

    def robot_torque_enable(self, cmd_type, name, enable):
        req = TorqueEnable.Request()
        req.cmd_type = cmd_type
        req.name = name
        req.enable = enable
        self._call_srv(self._srv_torque, req)

    def robot_reboot_motors(self, cmd_type, name, enable, smart_reboot=False):
        req = Reboot.Request()
        req.cmd_type = cmd_type
        req.name = name
        req.enable = enable
        req.smart_reboot = smart_reboot
        self._call_srv(self._srv_reboot, req)

    def robot_write_commands(self, group_name, commands):
        msg = JointGroupCommand()
        msg.name = group_name
        msg.cmd = list(commands)
        self.pub_group.publish(msg)

    def robot_write_joint_command(self, joint_name, command):
        msg = JointSingleCommand()
        msg.name = joint_name
        msg.cmd = command
        self.pub_single.publish(msg)

    def robot_write_trajectory(self, cmd_type, name, type, raw_traj):
        from builtin_interfaces.msg import Duration
        traj = JointTrajectory()
        for point in raw_traj:
            for key, value in point.items():
                traj_point = JointTrajectoryPoint()
                if type == "position":
                    traj_point.positions = list(value)
                elif type == "velocity":
                    traj_point.velocities = list(value)
                sec = int(key)
                nsec = int(round((key - sec) * 1e9))
                traj_point.time_from_start = Duration(sec=sec, nanosec=nsec)
                traj.points.append(traj_point)
        msg = JointTrajectoryCommand()
        msg.cmd_type = cmd_type
        msg.name = name
        msg.traj = traj
        self.pub_traj.publish(msg)

    def robot_get_joint_states(self):
        with self.js_mutex:
            return copy.deepcopy(self.joint_states)

    def robot_get_single_joint_state(self, name):
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        joint_index = joint_states.name.index(name)
        return {
            "position": joint_states.position[joint_index],
            "velocity": joint_states.velocity[joint_index],
            "effort": joint_states.effort[joint_index],
        }

    def _joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg
