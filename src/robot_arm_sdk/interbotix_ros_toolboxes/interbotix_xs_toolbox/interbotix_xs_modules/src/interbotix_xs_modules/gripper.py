import time
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_modules.core import InterbotixRobotXSCore


class InterbotixGripperXS(object):
    def __init__(self, robot_model, gripper_name, robot_name=None, gripper_pressure=0.5, gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350, init_node=True):
        self.dxl = InterbotixRobotXSCore(robot_model, robot_name=robot_name, init_node=init_node)
        self.gripper = InterbotixGripperXSInterface(self.dxl, gripper_name, gripper_pressure, gripper_pressure_lower_limit, gripper_pressure_upper_limit)


class InterbotixGripperXSInterface(object):
    def __init__(self, core, gripper_name, gripper_pressure=0.5, gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350):
        self.core = core
        gripper_info = self.core.srv_get_info("single", gripper_name)
        if gripper_info.mode != "current" and gripper_info.mode != "pwm":
            core.node.get_logger().error("Please set the gripper's 'operating mode' to 'pwm' or 'current'.")
        self.gripper_moving = False
        self.gripper_command = JointSingleCommand()
        self.gripper_command.name = gripper_name
        self.gripper_pressure_lower_limit = gripper_pressure_lower_limit
        self.gripper_pressure_upper_limit = gripper_pressure_upper_limit
        self.gripper_value = gripper_pressure_lower_limit + (gripper_pressure * (gripper_pressure_upper_limit - gripper_pressure_lower_limit))
        self.left_finger_index = self.core.js_index_map[gripper_info.joint_names[0]]
        self.left_finger_lower_limit = gripper_info.joint_lower_limits[0]
        self.left_finger_upper_limit = gripper_info.joint_upper_limits[0]
        core.node.create_timer(0.02, self._gripper_state)
        print("Gripper Name: %s\nGripper Pressure: %d%%" % (gripper_name, gripper_pressure * 100))
        print("Initialized InterbotixGripperXSInterface!\n")

    def _gripper_state(self):
        if self.gripper_moving:
            with self.core.js_mutex:
                gripper_pos = self.core.joint_states.position[self.left_finger_index]
            if (self.gripper_command.cmd > 0 and gripper_pos >= self.left_finger_upper_limit) or (self.gripper_command.cmd < 0 and gripper_pos <= self.left_finger_lower_limit):
                self.gripper_command.cmd = 0.0
                self.core.pub_single.publish(self.gripper_command)
                self.gripper_moving = False

    def gripper_controller(self, effort, delay):
        self.gripper_command.cmd = float(effort)
        with self.core.js_mutex:
            gripper_pos = self.core.joint_states.position[self.left_finger_index]
        if (self.gripper_command.cmd > 0 and gripper_pos < self.left_finger_upper_limit) or (self.gripper_command.cmd < 0 and gripper_pos > self.left_finger_lower_limit):
            self.core.pub_single.publish(self.gripper_command)
            self.gripper_moving = True
            time.sleep(delay)

    def set_pressure(self, pressure):
        self.gripper_value = self.gripper_pressure_lower_limit + pressure * (self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit)

    def open(self, delay=1.0):
        self.gripper_controller(self.gripper_value, delay)

    def close(self, delay=1.0):
        self.gripper_controller(-self.gripper_value, delay)
