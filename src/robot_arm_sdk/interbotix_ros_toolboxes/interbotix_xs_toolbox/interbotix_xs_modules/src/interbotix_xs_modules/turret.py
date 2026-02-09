import time
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from interbotix_xs_modules.core import InterbotixRobotXSCore


class InterbotixTurretXS(object):
    def __init__(self, robot_model, robot_name=None, group_name="turret", pan_profile_type="time", pan_profile_velocity=2.0, pan_profile_acceleration=0.3, tilt_profile_type="time", tilt_profile_velocity=2.0, tilt_profile_acceleration=0.3, init_node=True):
        self.dxl = InterbotixRobotXSCore(robot_model, robot_name=robot_name, init_node=init_node)
        self.turret = InterbotixTurretXSInterface(self.dxl, group_name, pan_profile_type, pan_profile_velocity, pan_profile_acceleration, tilt_profile_type, tilt_profile_velocity, tilt_profile_acceleration)


class InterbotixTurretXSInterface(object):
    def __init__(self, core, group_name="turret", pan_profile_type="time", pan_profile_velocity=2.0, pan_profile_acceleration=0.3, tilt_profile_type="time", tilt_profile_velocity=2.0, tilt_profile_acceleration=0.3):
        self.core = core
        group_info = self.core.srv_get_info("group", group_name)
        self.group_name = group_name
        self.pan_name = group_info.joint_names[0]
        self.tilt_name = group_info.joint_names[1]
        pan_limits = [group_info.joint_lower_limits[0], group_info.joint_upper_limits[0]]
        tilt_limits = [group_info.joint_lower_limits[1], group_info.joint_upper_limits[1]]
        pan_position = self.core.joint_states.position[self.core.js_index_map[self.pan_name]]
        tilt_position = self.core.joint_states.position[self.core.js_index_map[self.tilt_name]]
        self.info = {
            self.pan_name: {"command": pan_position, "profile_type": pan_profile_type, "profile_velocity": pan_profile_velocity, "profile_acceleration": pan_profile_acceleration, "lower_limit": pan_limits[0], "upper_limit": pan_limits[1]},
            self.tilt_name: {"command": tilt_position, "profile_type": tilt_profile_type, "profile_velocity": tilt_profile_velocity, "profile_acceleration": tilt_profile_acceleration, "lower_limit": tilt_limits[0], "upper_limit": tilt_limits[1]},
        }
        self.change_profile(self.pan_name, pan_profile_type, pan_profile_velocity, pan_profile_acceleration)
        self.change_profile(self.tilt_name, tilt_profile_type, tilt_profile_velocity, tilt_profile_acceleration)
        print("Turret Group Name: %s\nPan Name: %s, Profile Type: %s, Profile Velocity: %.1f, Profile Acceleration: %.1f\nTilt Name: %s, Profile Type: %s, Profile Velocity: %.1f, Profile Acceleration: %.1f" % (group_name, self.pan_name, pan_profile_type, pan_profile_velocity, pan_profile_acceleration, self.tilt_name, tilt_profile_type, tilt_profile_velocity, tilt_profile_acceleration))
        print("Initialized InterbotixTurretXSInterface!\n")

    def set_trajectory_profile(self, joint_name, profile_velocity=None, profile_acceleration=None):
        if profile_velocity is not None and profile_velocity != self.info[joint_name]["profile_velocity"]:
            if self.info[joint_name]["profile_type"] == "velocity":
                self.core.srv_set_reg(cmd_type="single", name=joint_name, reg="Profile_Velocity", value=profile_velocity)
            else:
                self.core.srv_set_reg(cmd_type="single", name=joint_name, reg="Profile_Velocity", value=int(profile_velocity * 1000))
            self.info[joint_name]["profile_velocity"] = profile_velocity
        if profile_acceleration is not None and profile_acceleration != self.info[joint_name]["profile_acceleration"]:
            if self.info[joint_name]["profile_type"] == "velocity":
                self.core.srv_set_reg(cmd_type="single", name=joint_name, reg="Profile_Acceleration", value=profile_acceleration)
            else:
                self.core.srv_set_reg(cmd_type="single", name=joint_name, reg="Profile_Acceleration", value=int(profile_acceleration * 1000))
            self.info[joint_name]["profile_acceleration"] = profile_acceleration

    def move(self, joint_name, position, profile_velocity=None, profile_acceleration=None, blocking=True, delay=0):
        if self.info[joint_name]["lower_limit"] <= position <= self.info[joint_name]["upper_limit"]:
            self.set_trajectory_profile(joint_name, profile_velocity, profile_acceleration)
            msg = JointSingleCommand()
            msg.name = joint_name
            msg.cmd = position
            self.core.pub_single.publish(msg)
            self.info[joint_name]["command"] = position
            if self.info[joint_name]["profile_type"] == "time" and blocking:
                time.sleep(self.info[joint_name]["profile_velocity"])
            else:
                time.sleep(delay)
        else:
            self.core.node.get_logger().warn("Goal position is outside the %s joint's limits." % joint_name)

    def pan(self, position, profile_velocity=None, profile_acceleration=None, blocking=True, delay=0):
        self.move(self.pan_name, position, profile_velocity, profile_acceleration, blocking, delay)

    def tilt(self, position, profile_velocity=None, profile_acceleration=None, blocking=True, delay=0):
        self.move(self.tilt_name, position, profile_velocity, profile_acceleration, blocking, delay)

    def pan_tilt_go_home(self, pan_profile_velocity=None, pan_profile_acceleration=None, tilt_profile_velocity=None, tilt_profile_acceleration=None, blocking=True, delay=0):
        self.pan_tilt_move(0, 0, pan_profile_velocity, pan_profile_acceleration, tilt_profile_velocity, tilt_profile_acceleration, blocking, delay)

    def pan_tilt_move(self, pan_position, tilt_position, pan_profile_velocity=None, pan_profile_acceleration=None, tilt_profile_velocity=None, tilt_profile_acceleration=None, blocking=True, delay=0):
        if (self.info[self.pan_name]["lower_limit"] <= pan_position <= self.info[self.pan_name]["upper_limit"]) and (self.info[self.tilt_name]["lower_limit"] <= tilt_position <= self.info[self.tilt_name]["upper_limit"]):
            self.set_trajectory_profile(self.pan_name, pan_profile_velocity, pan_profile_acceleration)
            self.set_trajectory_profile(self.tilt_name, tilt_profile_velocity, tilt_profile_acceleration)
            msg = JointGroupCommand()
            msg.name = self.group_name
            msg.cmd = [pan_position, tilt_position]
            self.core.pub_group.publish(msg)
            self.info[self.pan_name]["command"] = pan_position
            self.info[self.tilt_name]["command"] = tilt_position
            if self.info[self.pan_name]["profile_type"] == "time" and self.info[self.tilt_name]["profile_type"] == "time" and blocking:
                time.sleep(max(self.info[self.pan_name]["profile_velocity"], self.info[self.tilt_name]["profile_velocity"]))
            else:
                time.sleep(delay)
        else:
            self.core.node.get_logger().warn("One or both goal positions are outside the limits!")

    def change_profile(self, joint_name, profile_type, profile_velocity, profile_acceleration):
        if profile_type == "velocity":
            self.core.srv_set_op_modes("single", joint_name, "position", "velocity", profile_velocity, profile_acceleration)
        else:
            self.core.srv_set_op_modes("single", joint_name, "position", "time", int(profile_velocity * 1000), int(profile_acceleration * 1000))
        self.info[joint_name]["profile_velocity"] = profile_velocity
        self.info[joint_name]["profile_acceleration"] = profile_acceleration
        self.info[joint_name]["profile_type"] = profile_type

    def get_command(self, joint_name):
        return self.info[joint_name]["command"]

    def get_joint_commands(self):
        return [self.info[self.pan_name]["command"], self.info[self.tilt_name]["command"]]
