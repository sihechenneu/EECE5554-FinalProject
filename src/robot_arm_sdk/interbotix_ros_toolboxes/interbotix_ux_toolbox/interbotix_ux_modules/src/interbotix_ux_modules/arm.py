"""ROS 2 port of Interbotix arm interface for Universal Factory Xarm (with Modern Robotics IK)."""
import math
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from interbotix_common_modules import angle_manipulation as ang
from interbotix_ux_modules import mr_descriptions as mrd
from interbotix_ux_modules.core import InterbotixRobotUXCore
from interbotix_ux_modules.gripper import InterbotixGripperUXInterface

try:
    import modern_robotics as mr
except ImportError:
    mr = None


class InterbotixManipulatorUX(object):
    """Standalone module to control a Universal Factory Xarm and Gripper (Modern Robotics IK)."""

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
        self.arm = InterbotixArmUXInterface(self.ux)
        if gripper_type == "gripper":
            self.gripper = InterbotixGripperUXInterface(self.ux, pulse_vel, pulse)


class InterbotixArmUXInterface(object):
    """Arm interface using Modern Robotics IK."""

    def __init__(self, core):
        self.core = core
        self.robot_des = getattr(mrd, self.core.robot_model)
        self.limits = {name: {"lower": -3.14, "upper": 3.14} for name in self.core.joint_names}
        self.get_urdf_limits()
        self.joint_commands = []
        self.index_map = dict(zip(self.core.joint_names, range(len(self.core.joint_names))))
        self.initial_guesses = list(self.robot_des.Guesses)
        self.hold_up_positions = [0] * self.core.dof
        self.hold_up_positions[-2] = -1.57
        if self.core.ee_offset is not None:
            T_bf = ang.poseToTransformationMatrix(self.core.ee_offset)
            self.robot_des.M = np.dot(self.robot_des.M, T_bf)
        self.capture_joint_positions()
        self.initial_guesses.append(self.joint_commands)
        self.core._node.get_logger().info("Initializing InterbotixArmUXInterface...")
        self.core._node.get_logger().info("Complete!")

    def get_urdf_limits(self):
        """Get joint limits from robot_description param if available."""
        try:
            rd = self.core._node.get_parameter("robot_description").value
            if isinstance(rd, str) and rd.strip():
                root = ET.fromstring(rd)
                for joint in root.findall(".//joint"):
                    name = joint.get("name")
                    if name in self.limits:
                        lim = joint.find("limit")
                        if lim is not None:
                            self.limits[name]["lower"] = float(lim.get("lower", -3.14))
                            self.limits[name]["upper"] = float(lim.get("upper", 3.14))
        except Exception:
            pass

    def command_positions(self, positions, vel=1.0, accel=5.0, mode=0):
        self.joint_commands = list(positions)
        if self.core.mode != mode:
            self.core.robot_smart_mode_reset(mode)
        if mode == 0:
            self.core.robot_move_joint(positions, vel, accel)
        elif mode == 1:
            self.core.robot_move_servoj(positions)
        if mr is not None:
            self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)

    def check_joint_limits(self, positions):
        theta_list = [round(elem * 1000) / 1000.0 for elem in positions]
        for i, name in enumerate(self.core.joint_names):
            if not (self.limits[name]["lower"] <= theta_list[i] <= self.limits[name]["upper"]):
                return False
        return True

    def check_single_joint_limit(self, joint_name, position):
        theta = round(position * 1000) / 1000.0
        return self.limits[joint_name]["lower"] <= theta <= self.limits[joint_name]["upper"]

    def move_to_preset_pose(self, preset_pose="Home", vel=1.0, accel=5.0):
        if preset_pose == "Home":
            positions = [0] * self.core.dof
        elif preset_pose == "Hold-Up":
            positions = self.hold_up_positions
        else:
            positions = [0] * self.core.dof
        self.command_positions(positions, vel, accel, 0)

    def go_to_home_pose(self, vel=1.0, accel=5.0):
        self.move_to_preset_pose("Home", vel, accel)

    def go_to_holdup_pose(self, vel=1.0, accel=5.0):
        self.move_to_preset_pose("Hold-Up", vel, accel)

    def set_joint_positions(self, positions, vel=1.0, accel=5.0, mode=0):
        if self.check_joint_limits(positions):
            self.command_positions(positions, vel, accel, mode)
            return True
        return False

    def set_single_joint_position(self, joint_name, position, vel=1.0, accel=5.0, mode=0):
        if not self.check_single_joint_limit(joint_name, position):
            return False
        self.joint_commands[self.index_map[joint_name]] = position
        self.command_positions(self.joint_commands, vel, accel, mode)
        return True

    def set_ee_pose_matrix(self, T_sd, custom_guess=None, execute=True, vel=1.0, accel=5.0, mode=0):
        if mr is None:
            self.core._node.get_logger().warn("modern_robotics not installed; IK unavailable")
            return None, False
        if custom_guess is None:
            guesses = list(self.initial_guesses)
            guesses[3] = self.joint_commands
        else:
            guesses = [custom_guess]
        theta_list = self.joint_commands
        for guess in guesses:
            theta_list, success = mr.IKinSpace(
                self.robot_des.Slist, self.robot_des.M, T_sd, guess, 0.0001, 0.0001
            )
            if success and self.check_joint_limits(theta_list):
                if execute:
                    self.command_positions(theta_list, vel, accel, mode)
                return theta_list, True
        self.core._node.get_logger().warn("No valid pose could be found")
        return theta_list, False

    def set_ee_pose_components(
        self, x=0, y=0, z=0, roll=math.pi, pitch=0, yaw=0,
        custom_guess=None, execute=True, vel=1.0, accel=5.0
    ):
        T_sd = ang.poseToTransformationMatrix([x, y, z, roll, pitch, yaw])
        return self.set_ee_pose_matrix(T_sd, custom_guess, execute, vel, accel, 0)

    def set_ee_cartesian_trajectory(
        self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, moving_time=2.0, wp_period=0.02
    ):
        if mr is None:
            return False
        rpy = ang.rotationMatrixToEulerAngles(self.T_sb[:3, :3])
        T_sy = np.identity(4)
        T_sy[:2, :2] = ang.yawToRotationMatrix(rpy[2])
        T_yb = np.dot(mr.TransInv(T_sy), self.T_sb)
        rpy = ang.rotationMatrixToEulerAngles(T_yb[:3, :3])
        N = int(moving_time / wp_period)
        inc = 1.0 / float(N)
        joint_traj = []
        joint_positions = list(self.joint_commands)
        success = True
        for i in range(N + 1):
            joint_traj.append(list(joint_positions))
            if i == N:
                break
            T_yb[:3, 3] += [inc * x, inc * y, inc * z]
            rpy[0] += inc * roll
            rpy[1] += inc * pitch
            rpy[2] += inc * yaw
            T_yb[:3, :3] = ang.eulerAnglesToRotationMatrix(rpy)
            T_sd = np.dot(T_sy, T_yb)
            theta_list, success = self.set_ee_pose_matrix(T_sd, joint_positions, False)
            if success:
                joint_positions = theta_list
            else:
                self.core._node.get_logger().info(
                    "%.1f%% of trajectory planned. Trajectory will not be executed." % (i / float(N) * 100)
                )
                break
        if success:
            if self.core.mode != 1:
                self.core.robot_smart_mode_reset(1)
            period = rclpy.duration.Duration(nanoseconds=int(wp_period * 1e9))
            for cmd in joint_traj:
                self.core.robot_move_servoj(cmd)
                self.core._node.get_clock().sleep_for(period)
            self.T_sb = T_sd
            self.joint_commands = joint_positions
        return success

    def get_joint_commands(self):
        return list(self.joint_commands)

    def get_single_joint_command(self, joint_name):
        return self.joint_commands[self.index_map[joint_name]]

    def get_ee_pose_command(self):
        return np.array(self.T_sb) if hasattr(self, "T_sb") else np.identity(4)

    def get_ee_pose(self):
        if mr is None:
            return np.identity(4)
        joint_states = [
            self.core.joint_states.position[self.core.js_index_map[name]]
            for name in self.core.joint_names
        ]
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_states)

    def capture_joint_positions(self):
        self.joint_commands = [
            self.core.joint_states.position[self.core.js_index_map[name]]
            for name in self.core.joint_names
        ]
        if mr is not None:
            self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)
        else:
            self.T_sb = np.identity(4)
