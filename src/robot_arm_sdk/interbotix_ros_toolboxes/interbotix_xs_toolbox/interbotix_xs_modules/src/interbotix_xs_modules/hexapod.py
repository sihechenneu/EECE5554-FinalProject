# Hexapod module not yet ported to ROS 2 (depends on tf2_ros, rpi_modules, etc.).


class InterbotixHexapodXS(object):
    def __init__(self, *args, **kwargs):
        raise NotImplementedError("Hexapod module is not yet ported to ROS 2. Use arm, gripper, turret modules.")


class InterbotixHexapodXSInterface(object):
    def __init__(self, *args, **kwargs):
        raise NotImplementedError("Hexapod module is not yet ported to ROS 2.")
