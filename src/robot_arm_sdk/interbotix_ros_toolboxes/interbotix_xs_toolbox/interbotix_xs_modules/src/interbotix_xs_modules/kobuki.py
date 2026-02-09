# Kobuki module not yet ported to ROS 2 (depends on actionlib, move_base, kobuki_msgs).


class InterbotixKobukiInterface(object):
    def __init__(self, *args, **kwargs):
        raise NotImplementedError("Kobuki module is not yet ported to ROS 2. Use arm, gripper, turret modules.")
