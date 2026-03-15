#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from irobot_create_msgs.action import Dock, Undock
from core_interfaces.action import PickUp

from enum import Enum
import math
import numpy as np
import time

class MainState(Enum):
    UNDOCK = 0
    EXPLORE = 1
    APPROACH = 2
    PICK_UP = 3
    DEPOSIT = 4
    DOCK = 5
    CHARGING = 6

class DetectionFilter:
    def __init__(self, time_window=1.0, required_hits=5, max_variance=0.05):
        self.time_window = time_window
        self.required_hits = required_hits
        self.max_variance = max_variance
        self.detections = [] # list of (timestamp, x, y, z)

    def add_detection(self, current_time, x, y, z):
        self.detections.append((current_time, x, y, z))
        # Remove old ones
        self.detections = [d for d in self.detections if current_time - d[0] <= self.time_window]

    def get_stable_target(self):
        if len(self.detections) < self.required_hits:
            return None
            
        pts = np.array([[d[1], d[2], d[3]] for d in self.detections])
        var = np.var(pts, axis=0)
        if np.any(var > self.max_variance):
            return None
            
        # Return mean position
        mean_pt = np.mean(pts, axis=0)
        return mean_pt

class MissionStateMachine(Node):
    def __init__(self):
        super().__init__('mission_state_machine')
        
        self.state = MainState.UNDOCK
        
        # Parameters
        self.declare_parameter('deposit_x', 0.0)
        self.declare_parameter('deposit_y', 0.0)
        self.declare_parameter('low_battery_threshold', 0.15)
        self.declare_parameter('recharge_threshold', 0.90)
        self.declare_parameter('approach_distance', 0.3) # 30cm away from target
        
        # Filter
        self.filter = DetectionFilter(time_window=2.0, required_hits=3, max_variance=0.1)
        
        # Callbacks
        self.cb_group = ReentrantCallbackGroup()
        
        # Subscriptions
        self.create_subscription(Detection3DArray, '/yolo/spatial_detections', self.yolo_cb, 10, callback_group=self.cb_group)
        self.create_subscription(BatteryState, '/battery_state', self.battery_cb, 10, callback_group=self.cb_group)
        
        # Publishers
        self.explore_pub = self.create_publisher(Bool, '/explore/resume', 1)
        
        # Action Clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)
        self.pickup_client = ActionClient(self, PickUp, 'pickup_trash', callback_group=self.cb_group)
        self.undock_client = ActionClient(self, Undock, 'undock', callback_group=self.cb_group)
        self.dock_client = ActionClient(self, Dock, 'dock', callback_group=self.cb_group)
        
        # State variables
        self.target_position = None
        self.action_in_progress = False
        self.low_battery_triggered = False
        self._detection_count = 0    # total detections received (any state)
        self._explore_tick = 0       # ticks in EXPLORE state for heartbeat

        # Timer loop for state machine management
        self.timer = self.create_timer(1.0, self.state_machine_loop)
        self.get_logger().info('Mission State Machine initialized. State: UNDOCK')
        self.get_logger().info('  Subscribed to /yolo/spatial_detections for trash detection')
        self.get_logger().info('  Publishing /explore/resume to control frontier explorer')
        self.get_logger().info('  Using navigate_to_pose action for movement')
        
    def yolo_cb(self, msg):
        n = len(msg.detections)
        if n > 0:
            self._detection_count += n
            self.get_logger().debug(
                f'Received {n} detection(s) (total: {self._detection_count})'
            )

        if self.state != MainState.EXPLORE:
            return

        for det in msg.detections:
            if det.results:
                score = det.results[0].hypothesis.score
                cls = det.results[0].hypothesis.class_id
                if score > 0.5:
                    x = det.results[0].pose.pose.position.x
                    y = det.results[0].pose.pose.position.y
                    z = det.results[0].pose.pose.position.z

                    self.get_logger().info(
                        f'Detection: class="{cls}" score={score:.2f} '
                        f'pos=({x:.2f}, {y:.2f}, {z:.2f}) m'
                    )
                    self.filter.add_detection(time.time(), x, y, z)
                    stable_pos = self.filter.get_stable_target()

                    if stable_pos is not None and not self.action_in_progress:
                        self.get_logger().info(
                            f'Target locked at ({stable_pos[0]:.2f}, '
                            f'{stable_pos[1]:.2f}, {stable_pos[2]:.2f}) m — '
                            f'switching to APPROACH'
                        )
                        self.target_position = stable_pos
                        self.transition_to(MainState.APPROACH)
                        break
                        
    def battery_cb(self, msg):
        if msg.percentage <= 0.0:
            return
        low_threshold = self.get_parameter('low_battery_threshold').value
        recharge_threshold = self.get_parameter('recharge_threshold').value

        if msg.percentage < low_threshold and not self.low_battery_triggered:
            self.get_logger().warn(
                f'Low battery ({msg.percentage*100:.0f}%)! Initiating return to dock.'
            )
            self.low_battery_triggered = True
            self.transition_to(MainState.DOCK)

        elif self.state == MainState.CHARGING and msg.percentage >= recharge_threshold:
            self.get_logger().info(
                f'Battery recharged ({msg.percentage*100:.0f}%). Resuming mission.'
            )
            self.low_battery_triggered = False
            self.transition_to(MainState.UNDOCK)
            
    def transition_to(self, new_state):
        self.get_logger().info(f"Transitioning: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.action_in_progress = False
        
        if new_state not in (MainState.EXPLORE, MainState.UNDOCK, MainState.CHARGING):
            # Tell m-explore to pause
            msg = Bool()
            msg.data = False
            self.explore_pub.publish(msg)
            
    def state_machine_loop(self):
        if self.state == MainState.UNDOCK:
            if not self.action_in_progress:
                if not self.undock_client.server_is_ready():
                    self.get_logger().warn('Undock server not ready yet — retrying next tick.')
                    return
                self.action_in_progress = True
                self.get_logger().info('Undocking from charging station...')
                future = self.undock_client.send_goal_async(Undock.Goal())
                future.add_done_callback(self._undock_goal_response_callback)

        elif self.state == MainState.EXPLORE:
            self._explore_tick += 1
            if not self.action_in_progress:
                self.action_in_progress = True
                msg = Bool()
                msg.data = True
                self.explore_pub.publish(msg)
            # Heartbeat every 10 s so the user can verify the node is alive
            if self._explore_tick % 10 == 0:
                self.get_logger().info(
                    f'[EXPLORE] Running — {self._explore_tick} s elapsed, '
                    f'{self._detection_count} detections so far'
                )
                
        elif self.state == MainState.APPROACH:
            if not self.action_in_progress and self.target_position is not None:
                self.action_in_progress = True
                
                # Transform target_position to map frame dynamically here
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.pose.position.x = float(self.target_position[0])
                goal_msg.pose.pose.position.y = float(self.target_position[1]) - self.get_parameter('approach_distance').value
                goal_msg.pose.pose.orientation.w = 1.0
                
                self.get_logger().info("Sending nav goal to approach target...")
                if not self.nav_client.server_is_ready():
                    self.get_logger().warn("Nav2 server not ready — retrying next tick.")
                    self.action_in_progress = False
                    return
                self._send_nav_goal(goal_msg, MainState.PICK_UP, MainState.EXPLORE)
                
        elif self.state == MainState.PICK_UP:
            if not self.action_in_progress and self.target_position is not None:
                self.action_in_progress = True
                goal_msg = PickUp.Goal()
                goal_msg.target_position.x = float(self.target_position[0])
                goal_msg.target_position.y = float(self.target_position[1])
                goal_msg.target_position.z = float(self.target_position[2])
                
                self.get_logger().info("Sending pickup goal to arm controller...")
                if not self.pickup_client.server_is_ready():
                    self.get_logger().warn("PickUp server not ready — retrying next tick.")
                    self.action_in_progress = False
                    return
                self._send_pickup_goal(goal_msg)
                
        elif self.state == MainState.DEPOSIT:
            if not self.action_in_progress:
                self.action_in_progress = True
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.pose.position.x = self.get_parameter('deposit_x').value
                goal_msg.pose.pose.position.y = self.get_parameter('deposit_y').value
                goal_msg.pose.pose.orientation.w = 1.0
                
                self.get_logger().info("Navigating to designated trash deposit zone...")
                if not self.nav_client.server_is_ready():
                    self.get_logger().warn("Nav2 server not ready — retrying next tick.")
                    self.action_in_progress = False
                    return
                self._send_nav_goal(goal_msg, MainState.EXPLORE, MainState.EXPLORE)
                
        elif self.state == MainState.DOCK:
            if not self.action_in_progress:
                self.action_in_progress = True
                self.get_logger().info('Requesting dock_robot action...')
                if not self.dock_client.server_is_ready():
                    self.get_logger().warn('Dock server not ready — retrying next tick.')
                    self.action_in_progress = False
                    return
                future = self.dock_client.send_goal_async(Dock.Goal())
                future.add_done_callback(self._dock_goal_response_callback)
                
    def _dock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Dock goal rejected — retrying next tick.')
            self.action_in_progress = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dock_result_callback)

    def _dock_result_callback(self, future):
        self.get_logger().info('Docking complete. Waiting for battery to recharge...')
        self.transition_to(MainState.CHARGING)

    def _undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Undock goal rejected — assuming already undocked.')
            self.transition_to(MainState.EXPLORE)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._undock_result_callback)

    def _undock_result_callback(self, future):
        self.get_logger().info('Undocking complete. Starting exploration.')
        self.transition_to(MainState.EXPLORE)

    def _send_nav_goal(self, goal_msg, next_state_on_success, next_state_on_fail):
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda future_obj: self._nav_goal_response_callback(future_obj, next_state_on_success, next_state_on_fail)
        )

    def _nav_goal_response_callback(self, future, next_state_on_success, next_state_on_fail):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Nav goal rejected')
            self.transition_to(next_state_on_fail)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future_obj: self._nav_goal_result_callback(future_obj, next_state_on_success, next_state_on_fail)
        )
        
    def _nav_goal_result_callback(self, future, next_state_on_success, next_state_on_fail):
        self.get_logger().info(f'Nav goal finished. Transitioning to {next_state_on_success.name}')
        self.transition_to(next_state_on_success)
        
    def _send_pickup_goal(self, goal_msg):
        future = self.pickup_client.send_goal_async(goal_msg)
        future.add_done_callback(self._pickup_goal_response_callback)

    def _pickup_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Pickup goal rejected')
            self.transition_to(MainState.EXPLORE)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._pickup_goal_result_callback)
        
    def _pickup_goal_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Pickup successful! Transitioning to DEPOSIT')
            self.transition_to(MainState.DEPOSIT)
        else:
            self.get_logger().info('Pickup failed. Returning to EXPLORE')
            self.transition_to(MainState.EXPLORE)

def main(args=None):
    rclpy.init(args=args)
    node = MissionStateMachine()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
