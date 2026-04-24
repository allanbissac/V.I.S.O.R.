#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from package_with_vision_interfaces.msg import Object


class ObjectSubscriberNode(Node):
    COLOUR_NAME = {0: "red", 1: "green", 2: "blue"}
    TYPE_NAME = {0: "small cube", 1: "medium cube", 2: "bin"}

    def __init__(self):
        super().__init__("object_subscriber_node")
        self.create_subscription(Object, "/detected_object", self.cb, 10)
        self.get_logger().info("Subscribed to /detected_object (Object.msg).")

    def cb(self, msg: Object):
        if not msg.detected:
            self.get_logger().info("No object detected (received).")
            return

        colour = self.COLOUR_NAME.get(int(msg.colour), "unknown")
        obj_type = self.TYPE_NAME.get(int(msg.object_type), "unknown")

        # Required printout: x,y,z in mm + type index/name + colour code
        self.get_logger().info(
            f"Detected {obj_type} (index {msg.object_type}) | "
            f"colour={colour} (code {msg.colour}) | "
            f"x={msg.x:.1f}mm y={msg.y:.1f}mm z={msg.z:.1f}mm"
        )


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ObjectSubscriberNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
