import yaml
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
try:
    from tf_transformations import euler_from_quaternion
except ImportError:
    from tf_transformations import euler_from_quaternion


class Landmark(object):
    """A module used to store, configure, and publish data about landmarks."""

    def __init__(self, label=None, id_num=None, landmark_ns="landmarks", node=None):
        """
        Construct new landmark.

        :param label: name of this tracked landmark
        :param id: id of this landmark
        :param landmark_ns: namespace where the ROS parameters needed by the module are located
        :param node: rclpy node (required for TF buffer/listener in ROS 2)
        """
        self._node = node
        if node is not None:
            self.tf_buffer = Buffer()
            self.listener = TransformListener(self.tf_buffer, node)
        else:
            self.tf_buffer = None
            self.listener = None

        self.label_         = label
        self.id_            = id_num
        self.landmark_ns    = landmark_ns

        self.tf_wrt_cam = TransformStamped()
        self.tf_wrt_cam.child_frame_id = self.label_
        self.tf_wrt_map = TransformStamped()
        self.tf_wrt_map.child_frame_id = self.label_
        self.mounted_offset = 0.0

        self.tf_set_    = False
        self.mounted_   = False

        self._init_markers()

    def _time_now(self):
        if self._node is not None:
            return self._node.get_clock().now()
        return Time(seconds=0, nanoseconds=0)

    def get_id(self):
        return self.id_

    def set_id(self, id_num):
        self.id_ = id_num

    def is_set(self):
        return self.tf_set_

    def transform_to_new_frame(self, parent_old, parent_new):
        pose_old = PoseStamped()
        pose_old.pose.position = self.tf_wrt_cam.transform.translation
        pose_old.pose.orientation = self.tf_wrt_cam.transform.rotation
        pose_old.header.frame_id = parent_old
        pose_old.header.stamp = self._time_now().to_msg()
        try:
            pose_new = self.tf_buffer.transform(
                pose_old, parent_new, Duration(seconds=1))
            return pose_new
        except (LookupException, ConnectivityException, ExtrapolationException):
            raise

    def get_label(self):
        return self.label_

    def set_label(self, label):
        self.label_ = label

    def get_x(self):
        if self.is_set():
            return self.tf_wrt_map.transform.translation.x
        return 0.0

    def get_y(self):
        if self.is_set():
            return self.tf_wrt_map.transform.translation.y
        return 0.0

    def get_theta(self):
        if self.is_set():
            q = self.tf_wrt_map.transform.rotation
            return euler_from_quaternion(
                [q.x, q.y, q.z, q.w], 'rxyz')[2]
        return 0.0

    def set_tf_wrt_cam(self, pose):
        if isinstance(pose, Pose):
            self.tf_wrt_cam.transform.translation = pose.position
            self.tf_wrt_cam.transform.rotation = pose.orientation
        elif isinstance(pose, PoseStamped):
            self.tf_wrt_cam.transform.translation = pose.pose.position
            self.tf_wrt_cam.transform.rotation = pose.pose.orientation
        else:
            raise TypeError("Expected a Pose or PoseStamped.")

    def set_cam_frame_id(self, frame_id):
        self.tf_wrt_cam.header.frame_id = frame_id

    def get_tf_wrt_cam(self):
        return self.tf_wrt_cam

    def set_tf_wrt_map(self, pose, parent_new):
        if isinstance(pose, Pose):
            self.tf_wrt_map.transform.translation = pose.position
            self.tf_wrt_map.transform.rotation = pose.orientation
        elif isinstance(pose, PoseStamped):
            self.tf_wrt_map.transform.translation = pose.pose.position
            self.tf_wrt_map.transform.rotation = pose.pose.orientation
        else:
            raise TypeError("Expected a Pose or PoseStamped.")
        self.tf_wrt_map.header.stamp = self._time_now().to_msg()
        self.tf_wrt_map.header.frame_id = parent_new

    def get_tf_wrt_map(self):
        return self.tf_wrt_map

    def update_tfs(self, parent_old, parent_new):
        pose = self.transform_to_new_frame(parent_old, parent_new)
        self.set_tf_wrt_map(pose, parent_new)
        self.tf_set_ = True

    def set_mounted(self, mounted):
        self.mounted_ = mounted

    def set_mounted_offset(self, offset=0.0):
        self.mounted_offset = offset

    def is_mounted(self):
        return self.mounted_

    def get_mounted_offset(self):
        return self.mounted_offset

    def get_mb_goal(self):
        if not self.is_mounted():
            return [self.get_x(), self.get_y(), self.get_theta()]
        tf_goal = TransformStamped()
        tf_goal.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()
        tf_goal.header.frame_id = self.get_label()
        tf_goal.child_frame_id = "{}_goal".format(self.get_label())
        tf_goal.transform.translation.z = self.get_mounted_offset()
        tf_goal.transform.rotation.w = 1.0
        self.tf_buffer.set_transform(tf_goal, "default_authority")

        tf_map_to_goal = self.tf_buffer.lookup_transform(
            self.landmark_ns,
            tf_goal.child_frame_id,
            Time(seconds=0, nanoseconds=0))
        mb_goal = [
            tf_map_to_goal.transform.translation.x,
            tf_map_to_goal.transform.translation.y,
            euler_from_quaternion([
                tf_map_to_goal.transform.rotation.x,
                tf_map_to_goal.transform.rotation.y,
                tf_map_to_goal.transform.rotation.z,
                tf_map_to_goal.transform.rotation.w
            ])[2]
        ]
        return mb_goal

    def _init_markers(self):
        self.marker_sphere = Marker()
        self.marker_sphere.id = 0
        self.marker_sphere.header.frame_id = self.landmark_ns
        self.marker_sphere.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()
        self.marker_sphere.ns = self.get_label()
        self.marker_sphere.type = Marker.SPHERE
        self.marker_sphere.action = Marker.ADD
        self.marker_sphere.scale.x = 0.025
        self.marker_sphere.scale.y = 0.025
        self.marker_sphere.scale.z = 0.025
        self.marker_sphere.color.a = 1.0
        self.marker_sphere.color.r = 1.0
        self.marker_sphere.color.g = 0.0
        self.marker_sphere.color.b = 0.0
        self.marker_sphere.pose.position.z = 0.0

        self.marker_text = Marker()
        self.marker_text.id = 1
        self.marker_text.header.frame_id = self.landmark_ns
        self.marker_text.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()
        self.marker_text.ns = self.get_label()
        self.marker_text.type = Marker.TEXT_VIEW_FACING
        self.marker_text.text = "{}_goal".format(self.get_label())
        self.marker_text.scale.x = 0.05
        self.marker_text.scale.y = 0.05
        self.marker_text.scale.z = 0.05
        self.marker_text.color.a = 1.0
        self.marker_text.color.r = 1.0
        self.marker_text.color.g = 1.0
        self.marker_text.color.b = 1.0
        self.marker_text.pose.position.z = 0.05

    def __eq__(self, other):
        if isinstance(other, str):
            return self.label_ == other
        elif isinstance(other, int):
            return self.id_ == other
        if isinstance(other, Landmark):
            return (self.id_ == other.id_ or self.label_ == other.label_)
        return False

    def __repr__(self):
        tf = self.get_tf_wrt_map() if self.is_set() else self.get_tf_wrt_cam()
        return "---\nlabel: {}\nid: {}\ntf_set: {}\nmounted: {}\nmounted_offset: {}\ntf:\n{}\n---".format(
            self.label_, self.id_, self.is_set(), self.is_mounted(), self.mounted_offset, tf)


class LandmarkCollection(object):
    """Class to interact with Landmark data structure."""

    def __init__(self, landmarks=None, obs_frame=None, fixed_frame=None,
                 ros_on=False, init_node=False, node=None):
        self.data = landmarks if landmarks is not None else {}

        if init_node and node is None:
            rclpy.init()
            self._node = rclpy.create_node("landmark_collection")
        else:
            self._node = node

        if ros_on:
            self.ROS = True
            if self._node is None:
                raise ValueError("node must be provided when ros_on=True")
            self._node.declare_parameter("obs_frame", obs_frame or "camera_color_optical_frame")
            self._node.declare_parameter("fixed_frame", fixed_frame or "landmarks")
            self.obs_frame = self._node.get_parameter("obs_frame").value
            self.fixed_frame = self._node.get_parameter("fixed_frame").value

            self.static_tf_pub = self._node.create_publisher(
                TransformStamped, 'static_transforms', 10)
            self.marker_pub = self._node.create_publisher(
                MarkerArray, 'marker_viz', 10)
        else:
            self.ROS = False
            self.obs_frame = obs_frame
            self.fixed_frame = fixed_frame

    def get_landmark(self, id_num):
        if id_num is None:
            return list(self.data.values())
        elif isinstance(id_num, list):
            return [self.data[x] for x in id_num]
        return self.data[id_num]

    def add_landmark(self, label, id_num):
        self.data[id_num] = Landmark(label=label, id_num=id_num, node=self._node)

    def remove_landmark(self, id_num):
        self.data.pop(id_num)

    def get_valid_tags(self):
        self.update_valid_tags()
        return self.valid_tags

    def update_valid_tags(self):
        self.valid_tags = [l.get_id() for l in self.data.values()]

    def get_set_tags(self):
        return [l.get_id() for l in self.data.values() if l.is_set()]

    def get_set_landmarks(self):
        return [l for l in self.data.values() if l.is_set()]

    def save(self, filepath, ids=None):
        if self.is_empty():
            return True
        lm_yaml = {}
        for lm in self.get_landmark(ids):
            tf_map = lm.get_tf_wrt_map()
            lm_dict = {}
            lm_dict["id"] = lm.id_
            lm_dict["label"] = lm.label_
            lm_dict["set"] = lm.is_set()
            lm_dict["mounted"] = lm.is_mounted()
            lm_dict["mounted_offset"] = lm.mounted_offset
            if lm.is_set():
                lm_dict["tf"] = {}
                lm_dict["tf"]["frame_id"] = tf_map.header.frame_id
                lm_dict["tf"]["child_frame_id"] = tf_map.child_frame_id
                lm_dict["tf"]["x"] = float(tf_map.transform.translation.x)
                lm_dict["tf"]["y"] = float(tf_map.transform.translation.y)
                lm_dict["tf"]["z"] = float(tf_map.transform.translation.z)
                lm_dict["tf"]["qx"] = float(tf_map.transform.rotation.x)
                lm_dict["tf"]["qy"] = float(tf_map.transform.rotation.y)
                lm_dict["tf"]["qz"] = float(tf_map.transform.rotation.z)
                lm_dict["tf"]["qw"] = float(tf_map.transform.rotation.w)
            lm_yaml[lm.id_] = lm_dict
        with open(filepath, "w") as yamlfile:
            yaml.dump(lm_yaml, yamlfile, default_flow_style=False)
        if self._node:
            self._node.get_logger().info("Saved landmarks to {}".format(filepath))
        return True

    def load(self, filepath):
        try:
            with open(filepath, "r") as yamlfile:
                lm_dict = yaml.safe_load(yamlfile)
            if lm_dict is None:
                return True
            m = False
            mo = 0.0
            for key in lm_dict.keys():
                self.add_landmark(
                    label=lm_dict[key]["label"],
                    id_num=key)
                if "tf" in lm_dict[key].keys():
                    self.data[key].tf_wrt_map = TransformStamped()
                    self.data[key].tf_wrt_map.header.stamp = self._node.get_clock().now().to_msg() if self._node else Time(seconds=0, nanoseconds=0).to_msg()
                    self.data[key].tf_wrt_map.header.frame_id = lm_dict[key]["tf"]["frame_id"] if lm_dict[key]["tf"]["frame_id"] else self.obs_frame
                    self.data[key].tf_wrt_map.child_frame_id = lm_dict[key]["tf"]["child_frame_id"]
                    self.data[key].tf_wrt_map.transform.translation.x = lm_dict[key]["tf"]["x"]
                    self.data[key].tf_wrt_map.transform.translation.y = lm_dict[key]["tf"]["y"]
                    self.data[key].tf_wrt_map.transform.translation.z = lm_dict[key]["tf"]["z"]
                    self.data[key].tf_wrt_map.transform.rotation.x = lm_dict[key]["tf"]["qx"]
                    self.data[key].tf_wrt_map.transform.rotation.y = lm_dict[key]["tf"]["qy"]
                    self.data[key].tf_wrt_map.transform.rotation.z = lm_dict[key]["tf"]["qz"]
                    self.data[key].tf_wrt_map.transform.rotation.w = lm_dict[key]["tf"]["qw"]
                    self.data[key].tf_set_ = True
                else:
                    self.data[key].tf_wrt_cam.header.frame_id = self.obs_frame
                    self.data[key].tf_wrt_cam.child_frame_id = self.data[key].get_label()
                    self.data[key].tf_set_ = False
                if "mounted" in lm_dict[key].keys():
                    m = lm_dict[key]["mounted"]
                    mo = lm_dict[key]["mounted_offset"]
                self.data[key].mounted_ = m
                self.data[key].set_mounted_offset(mo)
            self.update_valid_tags()
            if self.ROS:
                self.pub_tfs(self.valid_tags)
                rclpy.spin_once(self._node, timeout_sec=1.0)
                self.pub_markers(self.valid_tags)
            return True
        except IOError as e:
            if self._node:
                self._node.get_logger().error(
                    "Error opening '{}': {}. No landmarks loaded.".format(filepath, e))
            return False

    def is_empty(self):
        return self.data == {}

    def pub_tfs(self, tag_ids=None):
        if self.is_empty():
            return True
        if tag_ids is None:
            for lm in self.data.values():
                if lm.is_set():
                    self.static_tf_pub.publish(lm.get_tf_wrt_map())
            return True
        for lm in [self.data[x] for x in tag_ids]:
            if lm.is_set():
                self.static_tf_pub.publish(lm.get_tf_wrt_map())
        return True

    def update_markers(self):
        for lm in self.get_set_landmarks():
            g = lm.get_mb_goal()
            lm.marker_sphere.pose.position.x = g[0]
            lm.marker_sphere.pose.position.y = g[1]
            lm.marker_sphere.pose.position.z = 0.1
            lm.marker_sphere.pose.orientation.x = 0.0
            lm.marker_sphere.pose.orientation.y = 0.0
            lm.marker_sphere.pose.orientation.z = 0.0
            lm.marker_sphere.pose.orientation.w = 1.0
            lm.marker_sphere.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()
            lm.marker_text.pose.position.x = g[0]
            lm.marker_text.pose.position.y = g[1]
            lm.marker_text.pose.position.z = 0.15
            lm.marker_text.pose.orientation.x = 0.0
            lm.marker_text.pose.orientation.y = 0.0
            lm.marker_text.pose.orientation.z = 0.0
            lm.marker_text.pose.orientation.w = 1.0
            lm.marker_text.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()

    def pub_markers(self, tag_ids):
        if not self.ROS:
            if self._node:
                self._node.get_logger().warn("Tried to publish marker but node is not active.")
            return
        self.update_markers()
        msg = MarkerArray()
        for lm in self.get_set_landmarks():
            if lm.get_id() in tag_ids:
                msg.markers.append(lm.marker_sphere)
                msg.markers.append(lm.marker_text)
        self.marker_pub.publish(msg)

    def __repr__(self):
        return str(list(self.data.values()))

    def __len__(self):
        return len(self.data)
