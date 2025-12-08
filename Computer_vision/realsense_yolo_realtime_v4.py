import rclpy
from rclpy.node import Node

from package_with_interfaces.srv import AddPoints   # <-- using your existing srv

import numpy as np
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO
import open3d as o3d
import time


class RealSenseXYZService(Node):

    def __init__(self):
        super().__init__("realsense_xyz_service_server")

        # -----------------------------
        # Setup service server
        # -----------------------------
        self.srv = self.create_service(
            AddPoints,
            "/get_xyz",
            self.get_xyz_callback
        )
        self.get_logger().info("Service /get_xyz READY.")

        # -----------------------------
        # Object for storing latest XYZ
        # -----------------------------
        self.latest_xyz = (float('nan'), float('nan'), float('nan'))

        # -----------------------------
        # Load YOLO model
        # -----------------------------
        MODEL_PATH = "/home/student10/Robotics_Systems_Design_Project/rgbd_vision/best.pt"
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info("YOLO model loaded.")

        # -----------------------------
        # RealSense initialization
        # -----------------------------
        self.pipeline = rs.pipeline()
        cfg = rs.config()

        WIDTH, HEIGHT, FPS = 640, 480, 30
        cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
        cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

        profile = self.pipeline.start(cfg)
        self.intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.align = rs.align(rs.stream.color)

        # -----------------------------
        # Timer → continuously process camera frames
        # -----------------------------
        self.timer = self.create_timer(0.03, self.process_frame)  # ~30 FPS

    # -------------------------------------------------------
    # Utility: convert pixel + depth → 3D point in meters
    # -------------------------------------------------------
    def pixel_to_point(self, px, py, depth_m):
        return rs.rs2_deproject_pixel_to_point(self.intr, [float(px), float(py)], float(depth_m))

    # -------------------------------------------------------
    # Main processing loop (runs at ~30 FPS)
    # -------------------------------------------------------
    def process_frame(self):

        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        color_img = np.asanyarray(color_frame.get_data())

        # ----------------- YOLO Detection -----------------
        results = self.model.predict(
            source=color_img,
            imgsz=640,
            conf=0.35,
            verbose=False
        )

        if len(results) == 0 or len(results[0].boxes) == 0:
            self.latest_xyz = (float('nan'), float('nan'), float('nan'))
            return

        # We only take the FIRST detection for simplicity
        box = results[0].boxes[0]
        x1, y1, x2, y2 = map(int, box.xyxy.cpu().numpy()[0])
        cx, cy = (x1 + x2)//2, (y1 + y2)//2

        depth_m = depth_frame.get_distance(cx, cy)
        if depth_m <= 0:
            return

        # Convert to 3D XYZ
        X, Y, Z = self.pixel_to_point(cx, cy, depth_m)

        self.latest_xyz = (float(X), float(Y), float(Z))

    # -------------------------------------------------------
    # ROS2 SERVICE CALLBACK
    # -------------------------------------------------------
    def get_xyz_callback(self, request, response):
        """
        Respond to /get_xyz request by returning the latest XYZ coordinate.
        """
        X, Y, Z = self.latest_xyz

        response.result.x = float(X)
        response.result.y = float(Y)
        response.result.z = float(Z)

        self.get_logger().info(f"Sent XYZ = ({X:.3f}, {Y:.3f}, {Z:.3f})")

        return response


def main(args=None):
    try:
        rclpy.init(args=args)
        node = RealSenseXYZService()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
