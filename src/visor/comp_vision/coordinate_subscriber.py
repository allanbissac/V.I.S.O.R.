#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String


class CentroidSubscriberNode(Node):
    def __init__(self):
        super().__init__("centroid_subscriber_node")
        self._latest_meta = []

        self.create_subscription(String, "/object_centroid_meta", self.meta_cb, 10)
        self.create_subscription(PoseArray, "/object_centroids", self.pose_cb, 10)

        self.get_logger().info("Subscribed to /object_centroids and /object_centroid_meta.")

    def meta_cb(self, msg: String):
        try:
            self._latest_meta = json.loads(msg.data)
        except Exception:
            self._latest_meta = []

    def pose_cb(self, msg: PoseArray):
        if not msg.poses:
            self.get_logger().info("No blobs detected (received empty PoseArray).")
            return

        for i, pose in enumerate(msg.poses):
            u = pose.position.x
            v = pose.position.y
            z = pose.position.z  # will be 0.0 in your XY-only publisher

            label = "unknown"
            if i < len(self._latest_meta) and isinstance(self._latest_meta[i], dict):
                label = self._latest_meta[i].get("label", "unknown")

            self.get_logger().info(f"{label}: x={u:.0f}, y={v:.0f}, z={z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = CentroidSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()