import math
import time
import numpy as np
import modern_robotics as mr
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand, JointTrajectoryCommand
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as BuiltinDuration
import interbotix_common_modules.angle_manipulation as ang
import interbotix_xs_modules.mr_descriptions as mrd
from interbotix_xs_modules.core import InterbotixRobotXSCore
from interbotix_xs_modules.gripper import InterbotixGripperXSInterface


class InterbotixManipulatorXS(object):
    def __init__(self, robot_model, group_name="arm", gripper_name="gripper", robot_name=None, moving_time=2.0, accel_time=0.3, gripper_pressure=0.5, gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350, init_node=True):
        self.dxl = InterbotixRobotXSCore(robot_model, robot_name=robot_name, init_node=init_node)
        self.arm = InterbotixArmXSInterface(self.dxl, robot_model, group_name, moving_time, accel_time)
        if gripper_name is not None:
            self.gripper = InterbotixGripperXSInterface(self.dxl, gripper_name, gripper_pressure, gripper_pressure_lower_limit, gripper_pressure_upper_limit)


class InterbotixArmXSInterface(object):
    def __init__(self, core, robot_model, group_name, moving_time=2.0, accel_time=0.3):
        self.core = core
        self.group_info = self.core.srv_get_info("group", group_name)
        if self.group_info.profile_type != "time":
            core.node.get_logger().error("Please set the group's 'profile type' to 'time'.")
        if self.group_info.mode != "position":
            core.node.get_logger().error("Please set the group's 'operating mode' to 'position'.")
        self.robot_des = getattr(mrd, robot_model)
        self.initial_guesses = [[0.0] * self.group_info.num_joints for i in range(3)]
        self.initial_guesses[1][0] = np.deg2rad(-120)
        self.initial_guesses[2][0] = np.deg2rad(120)
        self.moving_time = None
        self.accel_time = None
        self.group_name = group_name
        self.joint_commands = []
        self.rev = 2 * math.pi
        for name in self.group_info.joint_names:
            self.joint_commands.append(self.core.joint_states.position[self.core.js_index_map[name]])
        self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)
        self.set_trajectory_time(moving_time, accel_time)
        self.info_index_map = dict(zip(self.group_info.joint_names, range(self.group_info.num_joints)))
        print("Arm Group Name: %s\nMoving Time: %.2f seconds\nAcceleration Time: %.2f seconds\nDrive Mode: Time-Based-Profile" % (group_name, moving_time, accel_time))
        print("Initialized InterbotixArmXSInterface!\n")

    def publish_positions(self, positions, moving_time=None, accel_time=None, blocking=True):
        self.set_trajectory_time(moving_time, accel_time)
        self.joint_commands = list(positions)
        msg = JointGroupCommand()
        msg.name = self.group_name
        msg.cmd = self.joint_commands
        self.core.pub_group.publish(msg)
        if blocking:
            time.sleep(self.moving_time)
        self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)

    def set_trajectory_time(self, moving_time=None, accel_time=None):
        if moving_time is not None and moving_time != self.moving_time:
            self.moving_time = moving_time
            self.core.srv_set_reg(cmd_type="group", name=self.group_name, reg="Profile_Velocity", value=int(moving_time * 1000))
        if accel_time is not None and accel_time != self.accel_time:
            self.accel_time = accel_time
            self.core.srv_set_reg(cmd_type="group", name=self.group_name, reg="Profile_Acceleration", value=int(accel_time * 1000))

    def check_joint_limits(self, positions):
        theta_list = [int(elem * 1000) / 1000.0 for elem in positions]
        speed_list = [abs(goal - current) / float(self.moving_time) for goal, current in zip(theta_list, self.joint_commands)]
        for x in range(self.group_info.num_joints):
            if not (self.group_info.joint_lower_limits[x] <= theta_list[x] <= self.group_info.joint_upper_limits[x]):
                self.core.node.get_logger().warn("Would exceed position limits on joint %s." % x)
                self.core.node.get_logger().warn(
                    "Limits are [%f, %f], value was %f." % (self.group_info.joint_lower_limits[x], self.group_info.joint_upper_limits[x], theta_list[x])
                )
                return False
            if speed_list[x] > self.group_info.joint_velocity_limits[x]:
                self.core.node.get_logger().warn("Would exceed velocity limits on joint %s." % x)
                self.core.node.get_logger().warn("Limit is %f, value was %f." % (self.group_info.joint_velocity_limits[x], theta_list[x]))
                return False
        return True

    def check_single_joint_limit(self, joint_name, position):
        theta = int(position * 1000) / 1000.0
        speed = abs(theta - self.joint_commands[self.info_index_map[joint_name]]) / float(self.moving_time)
        ll = self.group_info.joint_lower_limits[self.info_index_map[joint_name]]
        ul = self.group_info.joint_upper_limits[self.info_index_map[joint_name]]
        vl = self.group_info.joint_velocity_limits[self.info_index_map[joint_name]]
        if not (ll <= theta <= ul):
            return False
        if speed > vl:
            return False
        return True

    def set_joint_positions(self, joint_positions, moving_time=None, accel_time=None, blocking=True):
        if self.check_joint_limits(joint_positions):
            self.publish_positions(joint_positions, moving_time, accel_time, blocking)
            return True
        return False

    def go_to_home_pose(self, moving_time=None, accel_time=None, blocking=True):
        self.publish_positions([0] * self.group_info.num_joints, moving_time, accel_time, blocking)

    def go_to_sleep_pose(self, moving_time=None, accel_time=None, blocking=True):
        self.publish_positions(list(self.group_info.joint_sleep_positions), moving_time, accel_time, blocking)

    def set_single_joint_position(self, joint_name, position, moving_time=None, accel_time=None, blocking=True):
        if not self.check_single_joint_limit(joint_name, position):
            return False
        self.set_trajectory_time(moving_time, accel_time)
        self.joint_commands[self.core.js_index_map[joint_name]] = position
        msg = JointSingleCommand()
        msg.name = joint_name
        msg.cmd = position
        self.core.pub_single.publish(msg)
        if blocking:
            time.sleep(self.moving_time)
        self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)
        return True

    def set_ee_pose_matrix(self, T_sd, custom_guess=None, execute=True, moving_time=None, accel_time=None, blocking=True):
        if custom_guess is None:
            initial_guesses = self.initial_guesses
        else:
            initial_guesses = [custom_guess]
        theta_list = []
        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(self.robot_des.Slist, self.robot_des.M, T_sd, guess, 0.001, 0.001)
            solution_found = True
            if success:
                for x in range(len(theta_list)):
                    if theta_list[x] <= -self.rev:
                        theta_list[x] %= -self.rev
                    elif theta_list[x] >= self.rev:
                        theta_list[x] %= self.rev
                    if round(theta_list[x], 3) < round(self.group_info.joint_lower_limits[x], 3):
                        theta_list[x] += self.rev
                    elif round(theta_list[x], 3) > round(self.group_info.joint_upper_limits[x], 3):
                        theta_list[x] -= self.rev
                solution_found = self.check_joint_limits(theta_list)
            else:
                solution_found = False
            if solution_found:
                if execute:
                    self.publish_positions(theta_list, moving_time, accel_time, blocking)
                    self.T_sb = T_sd
                return theta_list, True
        self.core.node.get_logger().warn("No valid pose could be found. Returned theta_list variable may be nonsense.")
        return theta_list, False

    def set_ee_pose_components(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=None, custom_guess=None, execute=True, moving_time=None, accel_time=None, blocking=True):
        if self.group_info.num_joints < 6 or (self.group_info.num_joints >= 6 and yaw is None):
            yaw = math.atan2(y, x)
        T_sd = np.identity(4)
        T_sd[:3, :3] = ang.eulerAnglesToRotationMatrix([roll, pitch, yaw])
        T_sd[:3, 3] = [x, y, z]
        return self.set_ee_pose_matrix(T_sd, custom_guess, execute, moving_time, accel_time, blocking)

    def set_ee_cartesian_trajectory(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, moving_time=None, wp_moving_time=0.2, wp_accel_time=0.1, wp_period=0.05):
        if self.group_info.num_joints < 6 and (y != 0 or yaw != 0):
            self.core.node.get_logger().info("Please leave the 'y' and 'yaw' fields at '0' when working with arms that have less than 6dof.")
            return False
        rpy = ang.rotationMatrixToEulerAngles(self.T_sb[:3, :3])
        T_sy = np.identity(4)
        T_sy[:3, :3] = ang.eulerAnglesToRotationMatrix([0.0, 0.0, rpy[2]])
        T_yb = np.dot(mr.TransInv(T_sy), self.T_sb)
        rpy[2] = 0.0
        if moving_time is None:
            moving_time = self.moving_time
        accel_time = self.accel_time
        N = int(moving_time / wp_period)
        inc = 1.0 / float(N)
        joint_traj = JointTrajectory()
        joint_positions = list(self.joint_commands)
        success = False
        T_sd = None
        for i in range(N + 1):
            joint_traj_point = JointTrajectoryPoint()
            joint_traj_point.positions = list(joint_positions)
            joint_traj_point.time_from_start = BuiltinDuration(sec=int(i * wp_period), nanosec=int(round((i * wp_period - int(i * wp_period)) * 1e9)))
            joint_traj.points.append(joint_traj_point)
            if i == N:
                break
            T_yb[:3, 3] += [inc * x, inc * y, inc * z]
            rpy[0] += inc * roll
            rpy[1] += inc * pitch
            rpy[2] += inc * yaw
            T_yb[:3, :3] = ang.eulerAnglesToRotationMatrix(rpy)
            T_sd = np.dot(T_sy, T_yb)
            theta_list, success = self.set_ee_pose_matrix(T_sd, joint_positions, False, blocking=False)
            if success:
                joint_positions = theta_list
            else:
                self.core.node.get_logger().info("%.1f%% of trajectory successfully planned. Trajectory will not be executed." % (i / float(N) * 100))
                break
        if success and T_sd is not None:
            self.set_trajectory_time(wp_moving_time, wp_accel_time)
            joint_traj.joint_names = list(self.group_info.joint_names)
            current_positions = []
            with self.core.js_mutex:
                for name in joint_traj.joint_names:
                    current_positions.append(self.core.joint_states.position[self.core.js_index_map[name]])
            joint_traj.points[0].positions = current_positions
            joint_traj.header.stamp = self.core.node.get_clock().now().to_msg()
            traj_cmd = JointTrajectoryCommand()
            traj_cmd.cmd_type = "group"
            traj_cmd.name = self.group_name
            traj_cmd.traj = joint_traj
            self.core.pub_traj.publish(traj_cmd)
            time.sleep(moving_time + wp_moving_time)
            self.T_sb = T_sd
            self.joint_commands = joint_positions
            self.set_trajectory_time(moving_time, accel_time)
        return success

    def set_relative_ee_position_wrt_to_base_frame(self, *, dx=0, dy=0, dz=0, custom_guess=None, execute=True, moving_time=None, accel_time=None, blocking=True):
        return self.set_ee_pose_components(
            x=self.T_sb[0, 3] + dx, y=self.T_sb[1, 3] + dy, z=self.T_sb[2, 3] + dz,
            custom_guess=custom_guess, execute=execute, moving_time=moving_time, accel_time=accel_time, blocking=blocking,
        )

    def get_joint_commands(self):
        return list(self.joint_commands)

    def get_single_joint_command(self, joint_name):
        return self.joint_commands[self.info_index_map[joint_name]]

    def get_ee_pose_command(self):
        return np.array(self.T_sb)

    def get_ee_pose(self):
        joint_states = [self.core.joint_states.position[self.core.js_index_map[name]] for name in self.group_info.joint_names]
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_states)

    def capture_joint_positions(self):
        self.joint_commands = []
        for name in self.group_info.joint_names:
            self.joint_commands.append(self.core.joint_states.position[self.core.js_index_map[name]])
        self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)
