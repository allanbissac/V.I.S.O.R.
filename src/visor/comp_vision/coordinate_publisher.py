#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs

from package_with_vision_interfaces.msg import Object


# -----------------------------
# Helpers from your algorithm
# -----------------------------
def get_median_distance(depth_frame, cx, cy, window_size=3):
    distances = []
    for dx in range(-window_size, window_size + 1):
        for dy in range(-window_size, window_size + 1):
            x = cx + dx
            y = cy + dy
            if 0 <= x < depth_frame.get_width() and 0 <= y < depth_frame.get_height():
                d = depth_frame.get_distance(x, y)
                if d > 0:
                    distances.append(d)
    return float(np.median(distances)) if distances else 0.0


def get_xyz_from_pixel(depth_frame, cx, cy, depth_value_m):
    intr = depth_frame.profile.as_video_stream_profile().intrinsics
    X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], depth_value_m)
    return float(X), float(Y), float(Z)  # meters


def estimate_volume_mm3(depth_frame, contour, depth_value_m):
    """
    Estimate cube side length and volume using contour size + depth.
    Returns: (side_m, volume_mm3)
    """
    rect = cv2.minAreaRect(contour)
    (_, _), (w_px, h_px), _ = rect

    intr = depth_frame.profile.as_video_stream_profile().intrinsics
    fx, fy = intr.fx, intr.fy

    width_m = (w_px * depth_value_m) / fx
    height_m = (h_px * depth_value_m) / fy
    side_m = (width_m + height_m) / 2.0

    volume_m3 = side_m ** 3
    volume_mm3 = volume_m3 * 1_000_000_000.0  # 1 m^3 = 1e9 mm^3
    return float(side_m), float(volume_mm3)


def clean_mask(mask, kernel):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def merge_nearby_contours(mask, min_area=200, merge_distance=17):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    merged = np.zeros_like(mask)

    for c in contours:
        if cv2.contourArea(c) > min_area:
            cv2.drawContours(merged, [c], -1, 255, thickness=cv2.FILLED)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (merge_distance, merge_distance))
    merged = cv2.dilate(merged, k, iterations=1)
    merged = cv2.morphologyEx(merged, cv2.MORPH_CLOSE, k)
    return merged


def smooth_point(prev_point, new_point, alpha=0.6):
    if prev_point is None:
        return new_point
    px, py = prev_point
    nx, ny = new_point
    sx = int(alpha * px + (1 - alpha) * nx)
    sy = int(alpha * py + (1 - alpha) * ny)
    return (sx, sy)


def smooth_value(prev_value, new_value, alpha=0.6):
    if prev_value is None:
        return new_value
    return alpha * prev_value + (1 - alpha) * new_value


def too_far(prev_point, new_point, max_jump=80):
    if prev_point is None:
        return False
    dx = new_point[0] - prev_point[0]
    dy = new_point[1] - prev_point[1]
    return math.hypot(dx, dy) > max_jump


def choose_stable_contour(contours, prev_center=None, min_area=700):
    valid = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not valid:
        return None
    if prev_center is None:
        return max(valid, key=cv2.contourArea)

    px, py = prev_center
    best = None
    best_score = float("inf")

    for c in valid:
        (x, y), _ = cv2.minEnclosingCircle(c)
        cx, cy = int(x), int(y)
        dist = math.hypot(cx - px, cy - py)
        area = cv2.contourArea(c)
        score = dist - 0.001 * area  # prefer proximity strongly, area weakly
        if score < best_score:
            best_score = score
            best = c
    return best


def detect_largest_blob(depth_frame, mask, label,
                        min_area=1000, min_radius=20):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < min_area:
        return None

    (x, y), radius = cv2.minEnclosingCircle(contour)
    cx, cy = int(x), int(y)
    radius = int(radius)
    if radius < min_radius:
        return None

    depth_m = get_median_distance(depth_frame, cx, cy, window_size=3)
    if depth_m <= 0:
        return None

    X, Y, Z = get_xyz_from_pixel(depth_frame, cx, cy, depth_m)
    side_m, volume_mm3 = estimate_volume_mm3(depth_frame, contour, depth_m)

    return {
        "label": label,
        "u": cx, "v": cy,
        "X_m": X, "Y_m": Y, "Z_m": Z,
        "volume_mm3": volume_mm3,
        "side_m": side_m,
        "radius": radius,
        "contour": contour,
    }


def detect_stable_blob(depth_frame, mask, label,
                       prev_center=None, prev_depth=None,
                       min_area=700, min_radius=12,
                       alpha_pos=0.6, alpha_depth=0.6,
                       max_jump_pixels=80):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = choose_stable_contour(contours, prev_center=prev_center, min_area=min_area)
    if contour is None:
        return None, prev_center, prev_depth

    (x, y), radius = cv2.minEnclosingCircle(contour)
    cx, cy = int(x), int(y)
    radius = int(radius)
    if radius < min_radius:
        return None, prev_center, prev_depth

    raw_center = (cx, cy)
    if too_far(prev_center, raw_center, max_jump=max_jump_pixels):
        if prev_center is not None:
            return None, prev_center, prev_depth

    cx, cy = smooth_point(prev_center, raw_center, alpha=alpha_pos)

    depth_m = get_median_distance(depth_frame, cx, cy, window_size=3)
    if depth_m <= 0:
        return None, prev_center, prev_depth

    depth_m = smooth_value(prev_depth, depth_m, alpha=alpha_depth)

    X, Y, Z = get_xyz_from_pixel(depth_frame, cx, cy, depth_m)
    side_m, volume_mm3 = estimate_volume_mm3(depth_frame, contour, depth_m)

    blob = {
        "label": label,
        "u": cx, "v": cy,
        "X_m": X, "Y_m": Y, "Z_m": Z,
        "volume_mm3": volume_mm3,
        "side_m": side_m,
        "radius": radius,
        "contour": contour,
    }
    return blob, (cx, cy), depth_m


# -----------------------------
# Publisher node
# -----------------------------
class VisionPublisherNode(Node):
    """
    Publishes /detected_object as package_with_vision_interfaces/msg/Object.

    Object.msg fields (your definition):
      x,y,z in mm
      colour: 0 red, 1 green, 2 blue
      object_type: 0 small cube, 1 medium cube, 2 bin
    """

    COLOUR_CODE = {"Red": 0, "Green": 1, "Blue": 2}

    TYPE_SMALL = 0
    TYPE_MEDIUM = 1
    TYPE_BIN = 2

    def __init__(self):
        super().__init__("centroid_publisher_node")

        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("show_debug", True)

        self.pub = self.create_publisher(Object, "/detected_object", 10)

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(cfg)

        self.align = rs.align(rs.stream.color)

        # Blue tracking state
        self.prev_blue_center = None
        self.prev_blue_depth = None

        self.show_debug = bool(self.get_parameter("show_debug").value)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1e-6), self.tick)

        self.get_logger().info("Publishing /detected_object (Object.msg) with XYZ(mm) + colour + object_type.")

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()

    @staticmethod
    def classify_object_type(volume_mm3: float) -> int:
        """
        Your requested thresholds:
          small cube (0): 35000..37000 mm^3
          medium cube (1): 79000..81000 mm^3
          bin (2): > 120000 mm^3
        """
        if 35000.0 <= volume_mm3 <= 37000.0:
            return VisionPublisherNode.TYPE_SMALL
        if 79000.0 <= volume_mm3 <= 81000.0:
            return VisionPublisherNode.TYPE_MEDIUM
        if volume_mm3 > 120000.0:
            return VisionPublisherNode.TYPE_BIN
        # default if not matching (choose what you want)
        return VisionPublisherNode.TYPE_BIN

    def publish_blob(self, blob):
        # Convert meters -> millimeters
        x_mm = blob["X_m"] * 1000.0
        y_mm = blob["Y_m"] * 1000.0
        z_mm = blob["Z_m"] * 1000.0
        vol_mm3 = blob["volume_mm3"]

        colour_code = int(self.COLOUR_CODE.get(blob["label"], -1))
        obj_type = int(self.classify_object_type(vol_mm3))

        msg = Object()
        msg.x = float(x_mm)
        msg.y = float(y_mm)
        msg.z = float(z_mm)
        msg.detected = True
        msg.colour = colour_code
        msg.object_type = obj_type

        self.pub.publish(msg)

        # Print required output on publisher terminal
        type_name = {0: "small cube", 1: "medium cube", 2: "bin"}.get(obj_type, "unknown")
        self.get_logger().info(
            f"{blob['label']} (colour={colour_code}) | "
            f"x={x_mm:.1f}mm y={y_mm:.1f}mm z={z_mm:.1f}mm | "
            f"volume={vol_mm3:.0f}mm^3 | object_type={type_name} (index {obj_type})"
        )

    def tick(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)

        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return

        frame = np.asanyarray(color_frame.get_data())

        # Blur before HSV
        frame_blur = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

        # Thresholds (same as your algorithm)
        lower_red1 = np.array([0, 100, 60]);   upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 60]); upper_red2 = np.array([180, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        lower_blue = np.array([95, 80, 40]);  upper_blue = np.array([145, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        lower_green = np.array([40, 80, 60]); upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        red_mask = clean_mask(red_mask, kernel)
        blue_mask = clean_mask(blue_mask, kernel)
        green_mask = clean_mask(green_mask, kernel)

        red_mask = merge_nearby_contours(red_mask, min_area=300, merge_distance=15)
        blue_mask = merge_nearby_contours(blue_mask, min_area=200, merge_distance=17)
        green_mask = merge_nearby_contours(green_mask, min_area=300, merge_distance=15)

        # Detect blobs
        red_blob = detect_largest_blob(depth_frame, red_mask, "Red")
        green_blob = detect_largest_blob(depth_frame, green_mask, "Green")
        blue_blob, self.prev_blue_center, self.prev_blue_depth = detect_stable_blob(
            depth_frame,
            blue_mask,
            "Blue",
            prev_center=self.prev_blue_center,
            prev_depth=self.prev_blue_depth,
            min_area=700,
            min_radius=12,
            alpha_pos=0.6,
            alpha_depth=0.6,
            max_jump_pixels=80,
        )

        detected = [b for b in [red_blob, green_blob, blue_blob] if b is not None]

        if not detected:
            msg = Object()
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
            msg.detected = False
            msg.colour = -1
            msg.object_type = self.TYPE_BIN
            self.pub.publish(msg)
            self.get_logger().info("No object detected.")
        else:
            for blob in detected:
                self.publish_blob(blob)

        if self.show_debug:
            cv2.imshow("RealSense Cube Detection with XYZ and Volume", frame)
            cv2.waitKey(1)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = VisionPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
