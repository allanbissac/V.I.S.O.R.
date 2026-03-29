import cv2
import numpy as np
import pyrealsense2 as rs
import math

# -----------------------------
# RealSense pipeline setup
# -----------------------------
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)
align = rs.align(rs.stream.color)

# -----------------------------
# Tracking state for blue blob
# -----------------------------
prev_blue_center = None
prev_blue_depth = None


def get_median_distance(depth_frame, cx, cy, window_size=3):
    """
    Return a stable depth reading by taking the median
    of valid depth values around the centroid.
    """
    distances = []

    for dx in range(-window_size, window_size + 1):
        for dy in range(-window_size, window_size + 1):
            x = cx + dx
            y = cy + dy

            if 0 <= x < depth_frame.get_width() and 0 <= y < depth_frame.get_height():
                d = depth_frame.get_distance(x, y)
                if d > 0:
                    distances.append(d)

    if len(distances) == 0:
        return 0.0

    return float(np.median(distances))


def get_xyz_from_pixel(depth_frame, cx, cy, depth_value):
    """
    Convert image pixel (cx, cy) + depth into 3D camera coordinates.
    Returns (x_m, y_m, z_m) in metres.
    """
    depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cx, cy], depth_value)
    return point_3d[0], point_3d[1], point_3d[2]


def estimate_cube_size_and_volume(depth_frame, contour, depth_value):
    """
    Estimate cube side length and volume from contour size and depth.

    Uses a rotated rectangle for a better estimate than a simple axis-aligned box.

    Returns:
        side_m, volume_m3, volume_cm3, w_px, h_px
    """
    rect = cv2.minAreaRect(contour)
    (_, _), (w_px, h_px), _ = rect

    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    fx = intrinsics.fx
    fy = intrinsics.fy

    width_m = (w_px * depth_value) / fx
    height_m = (h_px * depth_value) / fy

    side_m = (width_m + height_m) / 2.0
    volume_m3 = side_m ** 3
    volume_cm3 = volume_m3 * 1_000_000

    return side_m, volume_m3, volume_cm3, w_px, h_px


def clean_mask(mask, kernel):
    """
    Clean up a binary mask.
    """
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def merge_nearby_contours(mask, min_area=200, merge_distance=17):
    """
    Merge nearby contour regions by:
    1. keeping only contours above min_area
    2. drawing them filled onto a blank mask
    3. dilating slightly so close regions connect
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    merged_mask = np.zeros_like(mask)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            cv2.drawContours(merged_mask, [contour], -1, 255, thickness=cv2.FILLED)

    merge_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (merge_distance, merge_distance)
    )
    merged_mask = cv2.dilate(merged_mask, merge_kernel, iterations=1)
    merged_mask = cv2.morphologyEx(merged_mask, cv2.MORPH_CLOSE, merge_kernel)

    return merged_mask


def smooth_point(prev_point, new_point, alpha=0.6):
    """
    Exponential smoothing for 2D points.
    """
    if prev_point is None:
        return new_point

    px, py = prev_point
    nx, ny = new_point

    sx = int(alpha * px + (1 - alpha) * nx)
    sy = int(alpha * py + (1 - alpha) * ny)

    return (sx, sy)


def smooth_value(prev_value, new_value, alpha=0.6):
    """
    Exponential smoothing for scalar values.
    """
    if prev_value is None:
        return new_value
    return alpha * prev_value + (1 - alpha) * new_value


def too_far(prev_point, new_point, max_jump=80):
    """
    Reject sudden large jumps in detection.
    """
    if prev_point is None:
        return False

    dx = new_point[0] - prev_point[0]
    dy = new_point[1] - prev_point[1]
    return math.hypot(dx, dy) > max_jump


def choose_stable_contour(contours, prev_center=None, min_area=700):
    """
    Choose contour in a stable way.
    If previous centre exists, prefer the contour nearest to it.
    Otherwise choose the largest contour.
    """
    valid = [c for c in contours if cv2.contourArea(c) >= min_area]

    if not valid:
        return None

    if prev_center is None:
        return max(valid, key=cv2.contourArea)

    px, py = prev_center
    best_contour = None
    best_score = float("inf")

    for c in valid:
        (x, y), _ = cv2.minEnclosingCircle(c)
        cx, cy = int(x), int(y)

        distance = math.hypot(cx - px, cy - py)
        area = cv2.contourArea(c)

        # Prefer proximity strongly, area weakly
        score = distance - 0.001 * area

        if score < best_score:
            best_score = score
            best_contour = c

    return best_contour


def draw_blob_overlay(frame, cx, cy, radius, colour_bgr, label,
                      x_m, y_m, z_m, side_m, volume_cm3):
    """
    Draw blob outline, centroid, and text.
    """
    cv2.circle(frame, (cx, cy), radius, colour_bgr, 2)
    cv2.circle(frame, (cx, cy), 5, colour_bgr, -1)

    cv2.putText(
        frame,
        f"{label} u={cx} v={cy}",
        (cx - 70, cy - radius - 45),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        colour_bgr,
        2
    )

    cv2.putText(
        frame,
        f"X={x_m:.3f} Y={y_m:.3f} Z={z_m:.3f} m",
        (cx - 100, cy - radius - 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        colour_bgr,
        2
    )

    cv2.putText(
        frame,
        f"side={side_m:.3f} m vol={volume_cm3:.1f} cm^3",
        (cx - 120, cy - radius - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        colour_bgr,
        2
    )


def detect_largest_blob(frame, depth_frame, mask, label, colour_bgr,
                        min_area=1000, min_radius=20):
    """
    Standard largest-blob detector for red/green.
    Also estimates cube side length and volume.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)

    if area < min_area:
        return None

    (x, y), radius = cv2.minEnclosingCircle(contour)
    cx = int(x)
    cy = int(y)
    radius = int(radius)

    if radius < min_radius:
        return None

    depth_value = get_median_distance(depth_frame, cx, cy, window_size=3)
    if depth_value <= 0:
        return None

    x_m, y_m, z_m = get_xyz_from_pixel(depth_frame, cx, cy, depth_value)
    side_m, volume_m3, volume_cm3, w_px, h_px = estimate_cube_size_and_volume(
        depth_frame, contour, depth_value
    )

    draw_blob_overlay(frame, cx, cy, radius, colour_bgr, label,
                      x_m, y_m, z_m, side_m, volume_cm3)

    return {
        "u": cx,
        "v": cy,
        "x_m": round(x_m, 3),
        "y_m": round(y_m, 3),
        "z_m": round(z_m, 3),
        "colour": label,
        "radius": radius,
        "side_m": round(side_m, 3),
        "volume_m3": round(volume_m3, 6),
        "volume_cm3": round(volume_cm3, 1),
        "bbox_w_px": round(w_px, 1),
        "bbox_h_px": round(h_px, 1)
    }


def detect_stable_blob(frame, depth_frame, mask, label, colour_bgr,
                       prev_center=None, prev_depth=None,
                       min_area=700, min_radius=12,
                       alpha_pos=0.6, alpha_depth=0.6,
                       max_jump_pixels=80):
    """
    Stable detector for blue:
    - chooses contour nearest previous centre
    - smooths centroid
    - smooths depth
    - rejects sudden jumps
    - estimates cube side length and volume
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = choose_stable_contour(contours, prev_center=prev_center, min_area=min_area)

    if contour is None:
        return None, prev_center, prev_depth

    (x, y), radius = cv2.minEnclosingCircle(contour)
    cx = int(x)
    cy = int(y)
    radius = int(radius)

    if radius < min_radius:
        return None, prev_center, prev_depth

    raw_center = (cx, cy)

    if too_far(prev_center, raw_center, max_jump=max_jump_pixels):
        if prev_center is not None:
            return None, prev_center, prev_depth

    cx, cy = smooth_point(prev_center, raw_center, alpha=alpha_pos)

    depth_value = get_median_distance(depth_frame, cx, cy, window_size=3)
    if depth_value <= 0:
        return None, prev_center, prev_depth

    depth_value = smooth_value(prev_depth, depth_value, alpha=alpha_depth)

    x_m, y_m, z_m = get_xyz_from_pixel(depth_frame, cx, cy, depth_value)
    side_m, volume_m3, volume_cm3, w_px, h_px = estimate_cube_size_and_volume(
        depth_frame, contour, depth_value
    )

    draw_blob_overlay(frame, cx, cy, radius, colour_bgr, label,
                      x_m, y_m, z_m, side_m, volume_cm3)

    blob = {
        "u": cx,
        "v": cy,
        "x_m": round(x_m, 3),
        "y_m": round(y_m, 3),
        "z_m": round(z_m, 3),
        "colour": label,
        "radius": radius,
        "side_m": round(side_m, 3),
        "volume_m3": round(volume_m3, 6),
        "volume_cm3": round(volume_cm3, 1),
        "bbox_w_px": round(w_px, 1),
        "bbox_h_px": round(h_px, 1)
    }

    return blob, (cx, cy), depth_value


try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        # Blur before HSV conversion
        frame_blur = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

        # -----------------------------
        # HSV thresholds
        # -----------------------------

        # Red
        lower_red1 = np.array([0, 100, 60])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 60])
        upper_red2 = np.array([180, 255, 255])

        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = red_mask1 + red_mask2

        # Blue
        lower_blue = np.array([95, 80, 40])
        upper_blue = np.array([145, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Green
        lower_green = np.array([40, 80, 60])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # Morphology kernel
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        # Clean masks
        red_mask = clean_mask(red_mask, kernel)
        blue_mask = clean_mask(blue_mask, kernel)
        green_mask = clean_mask(green_mask, kernel)

        # Merge nearby contours
        red_mask = merge_nearby_contours(red_mask, min_area=300, merge_distance=15)
        blue_mask = merge_nearby_contours(blue_mask, min_area=200, merge_distance=17)
        green_mask = merge_nearby_contours(green_mask, min_area=300, merge_distance=15)

        # Detect red and green normally
        red_blob = detect_largest_blob(frame, depth_frame, red_mask, "Red", (0, 0, 255))
        green_blob = detect_largest_blob(frame, depth_frame, green_mask, "Green", (0, 255, 0))

        # Detect blue with temporal stabilisation
        blue_blob, prev_blue_center, prev_blue_depth = detect_stable_blob(
            frame,
            depth_frame,
            blue_mask,
            "Blue",
            (255, 0, 0),
            prev_center=prev_blue_center,
            prev_depth=prev_blue_depth,
            min_area=700,
            min_radius=12,
            alpha_pos=0.6,
            alpha_depth=0.6,
            max_jump_pixels=80
        )

        detected_blobs = []
        if red_blob is not None:
            detected_blobs.append(red_blob)
        if blue_blob is not None:
            detected_blobs.append(blue_blob)
        if green_blob is not None:
            detected_blobs.append(green_blob)

        print("Detected blobs:", detected_blobs)

        cv2.imshow("RealSense Cube Detection with XYZ and Volume", frame)
        # cv2.imshow("Blue Mask", blue_mask)
        # cv2.imshow("Red Mask", red_mask)
        # cv2.imshow("Green Mask", green_mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()