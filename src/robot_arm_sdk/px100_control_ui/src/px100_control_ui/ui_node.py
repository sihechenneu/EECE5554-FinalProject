#!/usr/bin/env python3
"""
ROS 2 node that bridges the PincherX 100 robot arm to a web UI.
Subscribes to joint_states, publishes joint commands, and runs a Flask server
that serves the UI and an API. All ROS calls run in the main thread; Flask
enqueues commands that are processed by a timer callback.
"""

import json
import queue
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from interbotix_xs_msgs.srv import TorqueEnable, RobotInfo

# Flask app (import here so we only need flask when running the node)
try:
    from flask import Flask, send_from_directory, request, jsonify
except ImportError:
    Flask = None


# Global bridge: set by main() before starting Flask thread; used by API routes
ros_bridge = None


def create_app(static_folder: Path):
    """Create Flask app with API and static file serving."""
    app = Flask(__name__, static_folder=str(static_folder), static_url_path='')
    app.config['JSON_SORT_KEYS'] = False

    @app.route('/')
    def index():
        return send_from_directory(app.static_folder, 'index.html')

    @app.route('/api/state')
    def api_state():
        """Return current joint states and connection status."""
        if ros_bridge is None:
            return jsonify({'error': 'ROS bridge not ready'}), 503
        return jsonify({
            'joint_states': getattr(ros_bridge, 'latest_joint_states', {}),
            'robot_name': getattr(ros_bridge, 'robot_name', 'px100'),
        })

    @app.route('/api/robot_info', methods=['GET'])
    def api_robot_info():
        """Return cached joint limits (fetched at startup in main thread)."""
        if ros_bridge is None:
            return jsonify({'error': 'ROS bridge not ready'}), 503
        cached = getattr(ros_bridge, 'cached_robot_info', None)
        if cached is None:
            return jsonify({'error': 'Robot info not yet loaded'}), 503
        return jsonify(cached)

    @app.route('/api/joint_single', methods=['POST'])
    def api_joint_single():
        """Publish a single joint command (enqueued for main thread)."""
        if ros_bridge is None:
            return jsonify({'error': 'ROS bridge not ready'}), 503
        data = request.get_json(force=True, silent=True) or {}
        name = data.get('name')
        cmd = data.get('cmd')
        if name is None or cmd is None:
            return jsonify({'error': 'name and cmd required'}), 400
        try:
            cmd = float(cmd)
        except (TypeError, ValueError):
            return jsonify({'error': 'cmd must be a number'}), 400
        ros_bridge.cmd_queue.put(('joint_single', name, cmd))
        return jsonify({'ok': True})

    @app.route('/api/joint_group', methods=['POST'])
    def api_joint_group():
        """Publish a group command (enqueued for main thread)."""
        if ros_bridge is None:
            return jsonify({'error': 'ROS bridge not ready'}), 503
        data = request.get_json(force=True, silent=True) or {}
        name = data.get('name')
        cmd = data.get('cmd')
        if name is None or cmd is None:
            return jsonify({'error': 'name and cmd (list) required'}), 400
        if not isinstance(cmd, list):
            return jsonify({'error': 'cmd must be a list'}), 400
        try:
            cmd = [float(x) for x in cmd]
        except (TypeError, ValueError):
            return jsonify({'error': 'cmd must be list of numbers'}), 400
        ros_bridge.cmd_queue.put(('joint_group', name, cmd))
        return jsonify({'ok': True})

    @app.route('/api/torque_enable', methods=['POST'])
    def api_torque_enable():
        """Call torque_enable service (enqueued)."""
        if ros_bridge is None:
            return jsonify({'error': 'ROS bridge not ready'}), 503
        data = request.get_json(force=True, silent=True) or {}
        cmd_type = data.get('cmd_type', 'group')
        name = data.get('name', 'arm')
        enable = data.get('enable', True)
        if isinstance(enable, str):
            enable = enable.lower() in ('true', '1', 'yes')
        ros_bridge.cmd_queue.put(('torque_enable', cmd_type, name, bool(enable)))
        return jsonify({'ok': True})

    return app


class Px100ControlUINode(Node):
    """ROS 2 node: joint_states subscriber, command publishers, service clients, timer to drain queue."""

    def __init__(self):
        super().__init__('px100_control_ui')
        self.declare_parameter('robot_name', 'px100')
        self.declare_parameter('ui_port', 8080)
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        p_port = self.get_parameter('ui_port').get_parameter_value()
        if p_port.type == Parameter.Type.INTEGER:
            self.ui_port = p_port.integer_value
        else:
            self.ui_port = int(p_port.string_value) if p_port.string_value else 8080
        self.latest_joint_states = {}
        self.cmd_queue = queue.Queue()

        ns = f'/{self.robot_name}'
        self.joint_states_sub = self.create_subscription(
            JointState,
            f'{ns}/joint_states',
            self._joint_states_cb,
            10,
        )
        self.pub_single = self.create_publisher(
            JointSingleCommand,
            f'{ns}/commands/joint_single',
            10,
        )
        self.pub_group = self.create_publisher(
            JointGroupCommand,
            f'{ns}/commands/joint_group',
            10,
        )
        self.torque_client = self.create_client(TorqueEnable, f'{ns}/torque_enable')
        self.robot_info_client_arm = self.create_client(RobotInfo, f'{ns}/get_robot_info')
        self.robot_info_client_gripper = self.create_client(RobotInfo, f'{ns}/get_robot_info')

        while not self.torque_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for torque_enable service...')
        while not self.robot_info_client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_robot_info service...')

        self.cached_robot_info = self._fetch_robot_info()
        if not self.cached_robot_info.get('joint_names'):
            # Fallback defaults for px100 (radians) if service didn't return
            self.cached_robot_info = {
                'joint_names': ['waist', 'shoulder', 'elbow', 'wrist_angle', 'gripper'],
                'joint_lower_limits': [-3.14, -1.94, -2.11, -1.75, 0.0],
                'joint_upper_limits': [3.14, 1.87, 1.61, 2.15, 0.8],
                'joint_sleep_positions': [0.0, -1.88, 1.5, 0.8, 0.0],
            }

        self.timer = self.create_timer(0.05, self._process_cmd_queue)
        self.get_logger().info(f'Px100 Control UI node started; robot_name={self.robot_name}')

    def _fetch_robot_info(self):
        """Call get_robot_info for arm and gripper; return dict for API."""
        joint_names, lower, upper, sleep = [], [], [], []
        req_arm = RobotInfo.Request()
        req_arm.cmd_type = 'group'
        req_arm.name = 'arm'
        future_arm = self.robot_info_client_arm.call_async(req_arm)
        rclpy.spin_until_future_complete(self, future_arm, timeout_sec=3.0)
        if future_arm.done() and future_arm.result() is not None:
            res = future_arm.result()
            joint_names.extend(res.joint_names)
            lower.extend(res.joint_lower_limits)
            upper.extend(res.joint_upper_limits)
            sleep.extend(res.joint_sleep_positions)
        req_g = RobotInfo.Request()
        req_g.cmd_type = 'single'
        req_g.name = 'gripper'
        future_g = self.robot_info_client_gripper.call_async(req_g)
        rclpy.spin_until_future_complete(self, future_g, timeout_sec=2.0)
        if future_g.done() and future_g.result() is not None:
            res = future_g.result()
            joint_names.append('gripper')
            lower.extend(res.joint_lower_limits)
            upper.extend(res.joint_upper_limits)
            sleep.extend(res.joint_sleep_positions if res.joint_sleep_positions else [0.0])
        return {
            'joint_names': joint_names,
            'joint_lower_limits': [float(x) for x in lower],
            'joint_upper_limits': [float(x) for x in upper],
            'joint_sleep_positions': [float(x) for x in sleep],
        }

    def _joint_states_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.latest_joint_states[name] = float(msg.position[i])

    def _process_cmd_queue(self):
        try:
            while True:
                item = self.cmd_queue.get_nowait()
                if item[0] == 'joint_single':
                    _, name, cmd = item
                    msg = JointSingleCommand()
                    msg.name = name
                    msg.cmd = cmd
                    self.pub_single.publish(msg)
                elif item[0] == 'joint_group':
                    _, name, cmd = item
                    msg = JointGroupCommand()
                    msg.name = name
                    msg.cmd = cmd
                    self.pub_group.publish(msg)
                elif item[0] == 'torque_enable':
                    _, cmd_type, name, enable = item
                    req = TorqueEnable.Request()
                    req.cmd_type = cmd_type
                    req.name = name
                    req.enable = enable
                    self.torque_client.call_async(req)
        except queue.Empty:
            pass


def main(args=None):
    global ros_bridge
    rclpy.init(args=args)
    node = Px100ControlUINode()
    ros_bridge = node

    # Locate web static folder (development: next to package; install: share/px100_control_ui/web)
    try:
        from ament_index_python.packages import get_package_share_directory
        static_folder = Path(get_package_share_directory('px100_control_ui')) / 'web'
    except Exception:
        static_folder = Path(__file__).resolve().parent.parent.parent / 'web'
    if not static_folder.is_dir():
        node.get_logger().warn(f'Web folder not found at {static_folder}; serving API only.')
        static_folder = Path(__file__).resolve().parent

    if Flask is None:
        node.get_logger().error('Flask not installed. pip install flask')
        return
    app = create_app(static_folder)
    port = node.ui_port
    # Run Flask in daemon thread so it doesn't block shutdown
    def run_flask():
        app.run(host='0.0.0.0', port=port, threaded=True, use_reloader=False)
    t = threading.Thread(target=run_flask, daemon=True)
    t.start()
    node.get_logger().info(f'UI available at http://localhost:{port}')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
