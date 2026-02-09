# Create3 module not yet ported to ROS 2 (depends on actionlib, move_base, irobot_create_msgs).


class InterbotixCreate3Interface(object):
    def __init__(self, *args, **kwargs):
        raise NotImplementedError("Create3 module is not yet ported to ROS 2. Use arm, gripper, turret modules.")
