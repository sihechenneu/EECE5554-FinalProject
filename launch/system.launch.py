#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    lc = LaunchContext()

    diagnostics_enable = EnvironmentVariable('TURTLEBOT4_DIAGNOSTICS', default_value='1')
    namespace = EnvironmentVariable('ROBOT_NAMESPACE', default_value='')

    # Package directories
    pkg_tb4_bringup = get_package_share_directory('turtlebot4_bringup')
    pkg_tb4_description = get_package_share_directory('turtlebot4_description')
    pkg_tb4_diagnostics = get_package_share_directory('turtlebot4_diagnostics')
    pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')
    pkg_oakd_bringup = get_package_share_directory('oakd_bringup')
    pkg_explore = get_package_share_directory('explore_lite')

    _this_dir = os.path.dirname(os.path.realpath(__file__))
    _cfg = os.path.join(_this_dir, '..', 'config')

    # Custom robot params: power_saver: false so turtlebot4_node does NOT call
    # oakd/stop_camera and interfere with our custom OAK-D launch.
    _custom_robot_yaml = os.path.join(_cfg, 'robot.yaml')

    # Custom Nav2 params: reduced costmap frequencies, obstacle_layer instead of
    # voxel_layer, fake_depth_scan observation source for door threshold detection.
    _custom_nav2_yaml = os.path.join(_cfg, 'nav2.yaml')

    # Custom SLAM params: coarser resolution and lower update rates to reduce RAM/CPU.
    _custom_slam_yaml = os.path.join(_cfg, 'slam.yaml')

    # -------------------------------------------------------------------------
    # TurtleBot4 base parameter file
    # -------------------------------------------------------------------------
    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=_custom_robot_yaml,
        description='TurtleBot4 parameter file (defaults to config/robot.yaml)',
    )
    param_file = LaunchConfiguration('param_file')

    namespaced_param_file = RewrittenYaml(
        source_file=param_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )

    # -------------------------------------------------------------------------
    # TurtleBot4 modules (all standard modules EXCEPT the stock OAK-D camera)
    # -------------------------------------------------------------------------
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_bringup, 'launch', 'robot.launch.py'])
        ),
        launch_arguments=[
            ('model', 'standard'),
            ('param_file', namespaced_param_file),
        ],
    )

    # joy_teleop omitted — no joystick connected; autonomous operation only.

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_bringup, 'launch', 'rplidar.launch.py'])
        ),
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_tb4_description, 'launch', 'robot_description.launch.py']
            )
        ),
        launch_arguments=[('model', 'standard')],
    )

    # -------------------------------------------------------------------------
    # SLAM — builds the map the moment the LiDAR scan comes in
    # -------------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'slam.launch.py'])
        ),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('sync', 'true'),
            ('params', _custom_slam_yaml),   # coarser res + lower update rates
        ],
    )

    # -------------------------------------------------------------------------
    # Nav2 — needs SLAM's /map, so give SLAM 5 s to publish the first map tile
    # -------------------------------------------------------------------------
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[system] Starting Nav2 navigation stack...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_tb4_nav, 'launch', 'nav2.launch.py'])
                ),
                launch_arguments=[
                    ('use_sim_time', 'false'),
                    ('params_file', _custom_nav2_yaml),  # reduced frequencies + fake_depth_scan
                ],
            ),
        ],
    )

    # -------------------------------------------------------------------------
    # m-explore — frontier exploration; needs Nav2 online, so delayed 15 s
    # Responds to /explore/resume (Bool) published by the mission state machine
    # -------------------------------------------------------------------------
    explore_node = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[system] Starting frontier explorer (m-explore)...'),
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                parameters=[
                    os.path.join(pkg_explore, 'config', 'params.yaml'),
                    {'use_sim_time': False},
                ],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            ),
        ],
    )

    # -------------------------------------------------------------------------
    # Custom OAK-D + on-device YOLO camera (replaces stock oakd.launch.py)
    # Delayed 45 s: Nav2 + SLAM + explore all settle first, leaving maximum
    # USB bandwidth and CPU headroom for the DepthAI pipeline init.
    # -------------------------------------------------------------------------
    oakd_yolo_launch = TimerAction(
        period=45.0,
        actions=[
            LogInfo(msg='[system] Starting OAK-D with YOLO configuration...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_oakd_bringup, 'launch', 'camera.launch.py'])
                ),
            ),
        ],
    )

    # -------------------------------------------------------------------------
    # depthimage_to_laserscan — converts OAK-D stereo depth to a 2-D LaserScan
    # that Nav2 uses to detect low obstacles (door thresholds, floor steps) which
    # the 2-D LiDAR cannot see.  Published to /oakd/fake_scan.
    # Started 5 s after the camera so the stereo topic is already live.
    # -------------------------------------------------------------------------
    fake_scan_node = TimerAction(
        period=50.0,
        actions=[
            LogInfo(msg='[system] Starting depthimage_to_laserscan (door threshold detector)...'),
            Node(
                package='depthimage_to_laserscan',
                executable='depthimage_to_laserscan_node',
                name='depthimage_to_laserscan',
                output='screen',
                remappings=[
                    ('depth', '/oakd/stereo/image_raw'),
                    ('depth_camera_info', '/oakd/stereo/camera_info'),
                    ('scan', '/oakd/fake_scan'),
                ],
                parameters=[{
                    'scan_time': 0.1,       # 10 Hz matches the stereo FPS
                    'scan_height': 20,      # sample 20 rows around image centre
                    'range_min': 0.15,
                    'range_max': 1.5,
                    'output_frame': 'oakd_left_camera_frame',
                }],
            ),
        ],
    )

    # -------------------------------------------------------------------------
    # Spatial calculator — lifts OAK-D 3D detections to /yolo/spatial_detections
    # Mission nodes — started after camera is confirmed live
    # -------------------------------------------------------------------------
    camera_dependent_nodes = TimerAction(
        period=50.0,
        actions=[
            LogInfo(msg='[system] Starting spatial calculator and mission nodes...'),
            Node(
                package='spatial_calculator',
                executable='spatial_calculator',
                name='spatial_calculator',
                output='screen',
                parameters=[{
                    'depth_topic': '/oakd/stereo/image_raw',
                    'info_topic': '/oakd/stereo/camera_info',
                    'yolo_topic': '/oakd/nn/spatial_detections',
                    'yolo_type': 'Detection3DArray',
                    'output_topic': '/yolo/spatial_detections',
                }],
            ),
            Node(
                package='core',
                executable='mission_state_machine',
                name='mission_state_machine',
                output='screen',
            ),
            Node(
                package='core',
                executable='px100_arm_controller',
                name='px100_arm_controller',
                output='screen',
            ),
        ],
    )

    # -------------------------------------------------------------------------
    # Assemble the full launch description
    # -------------------------------------------------------------------------
    tb4_actions = [
        PushRosNamespace(namespace),
        robot_launch,
        rplidar_launch,
        description_launch,
    ]

    if diagnostics_enable.perform(lc) == '1':
        tb4_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg_tb4_diagnostics, 'launch', 'diagnostics.launch.py']
                    )
                ),
                launch_arguments=[('namespace', namespace)],
            )
        )

    ld = LaunchDescription()
    ld.add_action(param_file_cmd)

    ld.add_action(LogInfo(msg='[system] ===== Autonomous Trash Collection System ====='))
    ld.add_action(LogInfo(msg='[system] Step 1/5 — Starting TurtleBot4 base modules (robot, LiDAR, description, teleop)'))
    ld.add_action(GroupAction(tb4_actions))

    ld.add_action(LogInfo(msg='[system] Step 2/5 — Starting SLAM (slam_toolbox online sync)'))
    ld.add_action(slam_launch)

    ld.add_action(LogInfo(msg='[system] Step 3/5 — Nav2 will start in 5 s (waits for first SLAM map tile)'))
    ld.add_action(nav2_launch)

    ld.add_action(LogInfo(msg='[system] Step 4/5 — m-explore will start in 15 s (waits for Nav2)'))
    ld.add_action(explore_node)

    ld.add_action(LogInfo(msg='[system] Step 5/5 — Spatial calc + mission nodes start at t=50 s (after OAK-D is live)'))
    ld.add_action(oakd_yolo_launch)
    ld.add_action(fake_scan_node)
    ld.add_action(camera_dependent_nodes)

    return ld
