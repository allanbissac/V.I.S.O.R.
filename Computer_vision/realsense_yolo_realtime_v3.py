import os
import sys

# CRITICAL: Remove OpenCV's Qt plugin path to avoid conflicts with ROS2/system Qt
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
os.environ.pop("QT_PLUGIN_PATH", None)
os.environ["QT_QPA_PLATFORM"] = "xcb"

import time
import numpy as np
import cv2
import pyrealsense2 as rs
import open3d as o3d
from ultralytics import YOLO

# ---------------- USER CONFIG ----------------
MODEL_PATH = "/home/student10/Robotics_Systems_Design_Project/rgbd_vision/best.pt"
CONF_THRESH = 0.35
IMG_SIZE = 640
DEPTH_PATCH = 5
DISPLAY_FPS = True

# Physical object dimensions (in meters)
CUBE_SIZE = 0.05  # 5 cm cubes
BIN_SIZE = 0.15   # 15 cm bins
SIZE_TOLERANCE = 0.02  # ±2 cm tolerance for size matching

# Size thresholds for classification
BLOCK_MAX_SIZE = 0.068  # Maximum dimension for blocks (8cm with tolerance)
BIN_MIN_SIZE = 0.14    # Minimum dimension for bins (12cm with tolerance)

# Color mapping for visualization (BGR format)
CLASS_COLORS = {
    'blue bin': (255, 100, 0),
    'blue cube': (255, 0, 0),
    'green bin': (100, 255, 0),
    'green cube': (0, 255, 0),
    'red bin': (0, 100, 255),
    'red cube': (0, 0, 255),
    'purple bin': (255, 100, 255),
    'purple cube': (255, 0, 255),
    'yellow bin': (100, 255, 255),
    'yellow cube': (0, 255, 255),
}

EXPECTED_SIZES = {
    'cube': CUBE_SIZE,
    'bin': BIN_SIZE
}
# ---------------------------------------------

def get_intrinsics(profile):
    """Extract camera intrinsics from RealSense profile."""
    video_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    return video_profile.get_intrinsics()

def median_depth_in_bbox(depth_frame, cx, cy, patch=5):
    """Get median depth in a patch around the center point."""
    half = patch // 2
    depths = []
    for dx in range(-half, half+1):
        for dy in range(-half, half+1):
            try:
                d = depth_frame.get_distance(cx+dx, cy+dy)
            except:
                d = 0
            if d > 0:
                depths.append(d)
    return np.median(depths) if len(depths) else float('nan')

def pixel_to_point(intrinsics, px, py, depth_m):
    """Convert 2D pixel + depth to 3D point in camera coordinates."""
    if np.isnan(depth_m) or depth_m <= 0:
        return (np.nan, np.nan, np.nan)
    return rs.rs2_deproject_pixel_to_point(intrinsics, [float(px), float(py)], float(depth_m))

def rs_frames_to_open3d(color_frame, depth_frame, intr):
    """Convert RealSense frames to Open3D point cloud."""
    depth_image = o3d.geometry.Image(np.asanyarray(depth_frame.get_data()))
    color_image = o3d.geometry.Image(np.asanyarray(color_frame.get_data()))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image,
        depth_image,
        depth_scale=1.0 / depth_frame.get_units(),  
        depth_trunc=3.0,
        convert_rgb_to_intensity=False
    )

    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intr.width, intr.height,
        intr.fx, intr.fy,
        intr.ppx, intr.ppy
    )

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    return pcd

def extract_object_point_cloud(pcd, x1, y1, x2, y2, intr, depth_frame):
    """
    Extract point cloud for a specific bounding box region.
    """
    if not pcd.has_points() or len(pcd.points) == 0:
        return None
    
    # Get depth values within bbox
    depths = []
    for y in range(y1, y2, max(1, (y2-y1)//10)):
        for x in range(x1, x2, max(1, (x2-x1)//10)):
            try:
                d = depth_frame.get_distance(x, y)
                if d > 0:
                    depths.append(d)
            except:
                pass
    
    if len(depths) == 0:
        return None
    
    # Get median depth for the object
    median_depth = np.median(depths)
    depth_threshold = 0.05  # 5cm threshold
    
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    
    valid_indices = []
    for i, point in enumerate(points):
        if point[2] <= 0:
            continue
        
        # Check if point is at similar depth
        if abs(point[2] - median_depth) > depth_threshold:
            continue
        
        # Project to 2D
        try:
            pixel = rs.rs2_project_point_to_pixel(
                [intr.fx, intr.fy, intr.ppx, intr.ppy, 0],
                point.tolist()
            )
            
            if x1 <= pixel[0] <= x2 and y1 <= pixel[1] <= y2:
                valid_indices.append(i)
        except:
            continue
    
    if len(valid_indices) == 0:
        return None
    
    object_pcd = pcd.select_by_index(valid_indices)
    return object_pcd

def get_object_dimensions(object_pcd):
    """
    Calculate real-world dimensions of an object from its point cloud.
    Returns dict with size measurements.
    """
    if object_pcd is None or not object_pcd.has_points() or len(object_pcd.points) < 10:
        return None
    
    try:
        # Get oriented bounding box (better for rotated objects)
        obb = object_pcd.get_oriented_bounding_box()
        obb_size = obb.extent  # [width, height, depth]
        
        # Get axis-aligned bounding box
        aabb = object_pcd.get_axis_aligned_bounding_box()
        aabb_size = aabb.get_extent()
        
        # Use OBB for size (more accurate for rotated objects)
        sorted_dims = np.sort(obb_size)  # Sort dimensions
        
        return {
            'obb_size': obb_size,
            'aabb_size': aabb_size,
            'center': np.asarray(obb.center),
            'max_dim': sorted_dims[2],  # Largest dimension
            'mid_dim': sorted_dims[1],  # Middle dimension
            'min_dim': sorted_dims[0],  # Smallest dimension
            'volume': obb_size[0] * obb_size[1] * obb_size[2],
            'num_points': len(object_pcd.points)
        }
    except Exception as e:
        print(f"Error computing dimensions: {e}")
        return None

def verify_object_type(cls_name, dimensions):
    """
    Verify if detected object type matches its measured size.
    Returns: (is_valid, object_type, size_error)
    """
    if dimensions is None:
        return False, "unknown", float('inf')
    
    max_dim = dimensions['max_dim']
    
    # Determine if it's a cube or bin from class name
    if 'cube' in cls_name.lower():
        expected_type = 'cube'
        expected_size = CUBE_SIZE
    elif 'bin' in cls_name.lower():
        expected_type = 'bin'
        expected_size = BIN_SIZE
    else:
        return False, "unknown", float('inf')
    
    # Calculate size error
    size_error = abs(max_dim - expected_size)
    
    # Check if size matches expectation
    is_valid = size_error <= SIZE_TOLERANCE
    
    # Additional check: cubes should be < BLOCK_MAX_SIZE, bins should be > BIN_MIN_SIZE
    if expected_type == 'cube':
        is_valid = is_valid and (max_dim <= BLOCK_MAX_SIZE)
    elif expected_type == 'bin':
        is_valid = is_valid and (max_dim >= BIN_MIN_SIZE)
    
    return is_valid, expected_type, size_error

def match_blocks_to_bins(detections_with_info):
    """
    Match cubes to their corresponding colored bins.
    Returns list of (cube, target_bin) pairs.
    """
    cubes = []
    bins = []
    
    # Separate cubes and bins (only include verified objects)
    for det in detections_with_info:
        if not det['size_verified']:
            continue
            
        cls_name = det['class'].lower()
        if 'cube' in cls_name:
            cubes.append(det)
        elif 'bin' in cls_name:
            bins.append(det)
    
    matches = []
    for cube in cubes:
        # Extract color from class name (e.g., 'blue cube' -> 'blue')
        cube_color = cube['class'].lower().split()[0]
        
        # Find matching bin with same color
        for bin_det in bins:
            bin_color = bin_det['class'].lower().split()[0]
            if cube_color == bin_color:
                distance_3d = np.linalg.norm(
                    np.array(cube['position_3d']) - np.array(bin_det['position_3d'])
                )
                matches.append({
                    'cube': cube,
                    'target_bin': bin_det,
                    'color': cube_color,
                    'distance': distance_3d
                })
                break
    
    return matches

def main():
    print(f"Loading YOLO model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print(f"Model classes: {model.names}")
    print(f"\nExpected sizes: Cubes={CUBE_SIZE*100:.1f}cm, Bins={BIN_SIZE*100:.1f}cm")
    print(f"Size tolerance: ±{SIZE_TOLERANCE*100:.1f}cm")

    # Initialize RealSense
    pipeline = rs.pipeline()
    cfg = rs.config()
    WIDTH, HEIGHT, FPS = 640, 480, 30

    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    profile = pipeline.start(cfg)

    intr = get_intrinsics(profile)
    align = rs.align(rs.stream.color)

    prev_time = time.time()
    frame_count = 0

    try:
        print("\nStarting detection loop... Press 'q' to quit\n" + "-"*80)
        
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())

            # ---- YOLO Detection ----
            results = model.predict(
                source=color_img,
                imgsz=IMG_SIZE,
                conf=CONF_THRESH,
                verbose=False
            )

            detections = []
            if len(results) and results[0].boxes is not None and len(results[0].boxes) > 0:
                for box in results[0].boxes:
                    xyxy = box.xyxy.cpu().numpy()[0]
                    conf = float(box.conf.cpu().numpy()[0])
                    cls_id = int(box.cls.cpu().numpy()[0])
                    cls_name = model.names[cls_id]
                    x1, y1, x2, y2 = map(int, xyxy)
                    detections.append((x1, y1, x2, y2, conf, cls_name))

            # ---- Create Point Cloud ----
            pcd = rs_frames_to_open3d(color_frame, depth_frame, intr)

            # ---- Process Detections ----
            detections_with_info = []
            valid_count = 0
            invalid_count = 0
            
            for (x1, y1, x2, y2, conf, cls_name) in detections:
                cx, cy = (x1+x2)//2, (y1+y2)//2
                
                # Get depth
                depth_m = median_depth_in_bbox(depth_frame, cx, cy, patch=DEPTH_PATCH)
                X, Y, Z = pixel_to_point(intr, cx, cy, depth_m)
                
                # Extract object point cloud
                object_pcd = extract_object_point_cloud(pcd, x1, y1, x2, y2, intr, depth_frame)
                
                # Get dimensions
                dimensions = get_object_dimensions(object_pcd)
                
                # Verify size matches expected type
                is_valid, obj_type, size_error = verify_object_type(cls_name, dimensions)
                
                if is_valid:
                    valid_count += 1
                else:
                    invalid_count += 1
                
                # Store detection info
                detection_info = {
                    'class': cls_name,
                    'confidence': conf,
                    'bbox': (x1, y1, x2, y2),
                    'center_2d': (cx, cy),
                    'position_3d': (X, Y, Z),
                    'depth': depth_m,
                    'dimensions': dimensions,
                    'size_verified': is_valid,
                    'object_type': obj_type,
                    'size_error': size_error,
                    'point_cloud': object_pcd
                }
                detections_with_info.append(detection_info)
                
                # Choose color
                color = CLASS_COLORS.get(cls_name, (255, 255, 0))
                
                # Use different color for invalid size detections
                if not is_valid:
                    color = (128, 128, 128)  # Gray for invalid
                
                # Build label
                label_parts = [cls_name, f"{conf:.2f}"]
                
                if not np.isnan(Z) and Z > 0:
                    label_parts.append(f"D:{Z:.3f}m")
                
                if dimensions is not None:
                    max_dim_cm = dimensions['max_dim'] * 100
                    label_parts.append(f"[{max_dim_cm:.1f}cm]")
                    
                    # Add verification status
                    if is_valid:
                        label_parts.append("✓")
                    else:
                        label_parts.append(f"✗ err:{size_error*100:.1f}cm")
                
                label = " ".join(label_parts)
                
                # Draw bounding box (thicker for valid detections)
                thickness = 3 if is_valid else 1
                cv2.rectangle(color_img, (x1, y1), (x2, y2), color, thickness)
                
                # Draw label with background
                (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(color_img, (x1, y1-label_h-10), (x1+label_w, y1), color, -1)
                cv2.putText(color_img, label, (x1, y1-5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Draw center point
                cv2.circle(color_img, (cx, cy), 4, color, -1)

            # ---- Match Cubes to Bins ----
            matches = match_blocks_to_bins(detections_with_info)
            
            # Print matches
            if matches and frame_count % 30 == 0:  # Print every 30 frames
                print(f"\n[Frame {frame_count}] Found {len(matches)} verified cube-bin matches:")
                for match in matches:
                    cube_size = match['cube']['dimensions']['max_dim'] * 100
                    bin_size = match['target_bin']['dimensions']['max_dim'] * 100
                    print(f"  {match['color'].upper()}: "
                          f"Cube({cube_size:.1f}cm)@({match['cube']['position_3d'][2]:.2f}m) -> "
                          f"Bin({bin_size:.1f}cm)@({match['target_bin']['position_3d'][2]:.2f}m) "
                          f"[dist: {match['distance']:.3f}m]")

            # ---- Display Statistics ----
            stats_y = 20
            cv2.putText(color_img, f"Total: {len(detections)} | Valid: {valid_count} | Invalid: {invalid_count}", 
                       (10, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            stats_y += 25
            cv2.putText(color_img, f"Matches: {len(matches)}", (10, stats_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # FPS
            frame_count += 1
            if DISPLAY_FPS:
                now = time.time()
                if now - prev_time >= 1:
                    fps = frame_count / (now - prev_time)
                    prev_time = now
                    frame_count = 0
                stats_y += 25
                cv2.putText(color_img, f"FPS: {fps:.1f}", (10, stats_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Block-Bin Detection System", color_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("\nShutdown complete.")

if __name__ == "__main__":
    main()