#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__("odom_tf_broadcaster")
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self.cb, 50)

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id if msg.child_frame_id else "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    try:
        rclpy.init()
        node = OdomTFBroadcaster()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()
