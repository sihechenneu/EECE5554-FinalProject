# Locobot module not yet ported to ROS 2 (depends on perception_modules, move_base, etc.).


class InterbotixLocobotXS(object):
    def __init__(self, *args, **kwargs):
        raise NotImplementedError("Locobot module is not yet ported to ROS 2. Use arm, gripper, turret modules.")
