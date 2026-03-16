#!/usr/bin/env python3
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose

import pyrealsense2 as rs


def draw_blob_info(mask, min_area=500):
    """
    Returns a list of centroids (cx, cy) for blobs in a binary mask.
    """
    centroids = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area <= min_area:
            continue

        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        centroids.append((cx, cy))

    return centroids, contours


class CentroidPublisherNode(Node):
    """
    Publishes 2D pixel centroids (x,y) of colored blobs.

    Topics:
      - /object_centroids (PoseArray): x=cx, y=cy, z=0.0 (pixel coordinates)
      - /object_centroid_meta (String): JSON list of labels in same order
    """

    def __init__(self):
        super().__init__("centroid_publisher_node")

        # ---- Parameters ----
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("show_debug", True)

        # RealSense stream parameters
        self.declare_parameter("rs_width", 640)
        self.declare_parameter("rs_height", 480)
        self.declare_parameter("rs_fps", 30)

        # Blob detection params
        self.declare_parameter("min_area", 500)

        self.frame_id = self.get_parameter("frame_id").value
        self.show_debug = bool(self.get_parameter("show_debug").value)

        # ---- Publishers ----
        self.pub_posearray = self.create_publisher(PoseArray, "/object_centroids", 10)
        self.pub_meta = self.create_publisher(String, "/object_centroid_meta", 10)

        # ---- RealSense pipeline (COLOR stream) ----
        self.rs_pipeline = rs.pipeline()
        cfg = rs.config()

        w = int(self.get_parameter("rs_width").value)
        h = int(self.get_parameter("rs_height").value)
        fps = int(self.get_parameter("rs_fps").value)

        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        self.profile = self.rs_pipeline.start(cfg)

        # Warm-up frames
        for _ in range(5):
            self.rs_pipeline.wait_for_frames()

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1e-6), self.tick)

        self.get_logger().info("Publishing /object_centroids (PoseArray) and /object_centroid_meta (JSON String).")

    def destroy_node(self):
        try:
            self.rs_pipeline.stop()
        except Exception:
            pass
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()

    def tick(self):
        min_area = int(self.get_parameter("min_area").value)

        frames = self.rs_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        frame = np.asanyarray(color_frame.get_data())  # BGR uint8

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # --- HSV masks (your exact thresholds) ---
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = red_mask1 + red_mask2

        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        lower_green = np.array([40, 70, 70])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        kernel = np.ones((5, 5), np.uint8)

        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        # --- Find centroids ---
        red_centroids, red_contours = draw_blob_info(red_mask, min_area=min_area)
        blue_centroids, blue_contours = draw_blob_info(blue_mask, min_area=min_area)
        green_centroids, green_contours = draw_blob_info(green_mask, min_area=min_area)

        # ---- Publish PoseArray (pixel xy) + meta ----
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        meta_list = []

        def add_centroids(label, centroids, color_bgr, contours):
            # draw debug
            if self.show_debug:
                cv2.drawContours(frame, contours, -1, color_bgr, 2)

            for (cx, cy) in centroids:
                p = Pose()
                p.position.x = float(cx)   # pixel u
                p.position.y = float(cy)   # pixel v
                p.position.z = 0.0         # no depth
                p.orientation.w = 1.0
                msg.poses.append(p)

                meta_list.append({"label": label, "cx": cx, "cy": cy})

                if self.show_debug:
                    cv2.circle(frame, (cx, cy), 5, color_bgr, -1)
                    cv2.putText(
                        frame,
                        f"{label} ({cx},{cy})",
                        (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color_bgr,
                        2
                    )

        add_centroids("red", red_centroids, (0, 0, 255), red_contours)
        add_centroids("blue", blue_centroids, (255, 0, 0), blue_contours)
        add_centroids("green", green_centroids, (0, 255, 0), green_contours)

        self.pub_posearray.publish(msg)

        meta_msg = String()
        meta_msg.data = json.dumps(meta_list)
        self.pub_meta.publish(meta_msg)

        if self.show_debug:
            cv2.imshow("Blob Contours and Centroids (RealSense Color)", frame)
            cv2.waitKey(1)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CentroidPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()