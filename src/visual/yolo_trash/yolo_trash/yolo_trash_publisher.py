import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image

class YoloTrashPublisher(Node):
    def __init__(self):
        super().__init__("yolo_trash_publisher")
        self.declare_parameter("bag_path", "")
        self.declare_parameter("image_topic_in_bag", "/oakd/rgb/image_raw")
        self.declare_parameter("image_topic_out", "/oakd/rgb/image_raw")
        self.declare_parameter("storage_id", "mcap")
        self.declare_parameter("rate", 10.0)

        bag_path = self.get_parameter("bag_path").value
        if not bag_path:
            self.get_logger().error("bag_path is required")
            raise ValueError("bag_path is required")
        self._image_topic_in_bag = self.get_parameter("image_topic_in_bag").value
        self._image_topic_out = self.get_parameter("image_topic_out").value
        storage_id = self.get_parameter("storage_id").value
        rate = self.get_parameter("rate").value

        self._pub = self.create_publisher(Image, self._image_topic_out, 10)
        self._reader = SequentialReader()
        self._reader.open(
            StorageOptions(uri=bag_path, storage_id=storage_id),
            ConverterOptions("", ""),
        )
        self._topic_types = {t.name: t.type for t in self._reader.get_all_topics_and_types()}
        if self._image_topic_in_bag not in self._topic_types:
            self.get_logger().error(f"Topic {self._image_topic_in_bag} not in bag. Available: {list(self._topic_types.keys())}")
            raise ValueError("image_topic_in_bag not found in bag")
        self._msg_type_str = self._topic_types[self._image_topic_in_bag]
        if self._msg_type_str != "sensor_msgs/msg/Image":
            self.get_logger().error(f"Topic is {self._msg_type_str}, expected sensor_msgs/msg/Image")
            raise ValueError("Unsupported message type")
        self._timer = self.create_timer(1.0 / rate, self._timer_cb)

    def _timer_cb(self):
        if not self._reader.has_next():
            return
        topic, data, timestamp = self._reader.read_next()
        if topic != self._image_topic_in_bag:
            return
        msg = deserialize_message(data, Image)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrashPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
