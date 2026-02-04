# realsense_yolo_realtime_v2

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

# Thresholds for your block/bin logic
BLOCK_MAX_SIZE = 0.06
BIN_MIN_SIZE = 0.145
PIXEL_AREA_THRESHOLD = 15000
# ---------------------------------------------

def get_intrinsics(profile):
    video_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    return video_profile.get_intrinsics()

def median_depth_in_bbox(depth_frame, cx, cy, patch=5):
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
    if np.isnan(depth_m) or depth_m <= 0:
        return (np.nan, np.nan, np.nan)
    return rs.rs2_deproject_pixel_to_point(intrinsics, [float(px), float(py)], float(depth_m))

# ------------- NEW: RealSense → Open3D conversion ----------------

def rs_frames_to_open3d(color_frame, depth_frame, intr):
    depth_image = o3d.geometry.Image(np.asanyarray(depth_frame.get_data()))
    color_image = o3d.geometry.Image(np.asanyarray(color_frame.get_data()))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image,
        depth_image,
        depth_scale=1.0 / depth_frame.get_units(),  
        depth_trunc=2.0,
        convert_rgb_to_intensity=False
    )

    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intr.width, intr.height,
        intr.fx, intr.fy,
        intr.ppx, intr.ppy
    )

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    return pcd

# ------------- FIXED: Plane removal + clustering with error handling -------------------

def segment_objects_from_pcd(pcd):
    """
    Segment objects from point cloud with proper error handling.
    Returns empty list if segmentation fails.
    """
    # Check if point cloud has enough points
    if not pcd.has_points() or len(pcd.points) < 10:
        print(f"Warning: Point cloud too small ({len(pcd.points)} points), skipping segmentation")
        return []
    
    # Downsample
    pcd_down = pcd.voxel_down_sample(voxel_size=0.005)
    
    # Check again after downsampling
    if len(pcd_down.points) < 3:
        print(f"Warning: After downsampling, only {len(pcd_down.points)} points remain")
        return []
    
    try:
        # Remove table plane
        plane_model, inliers = pcd_down.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=400
        )
        
        # Check if we have enough inliers for a valid plane
        if len(inliers) < 100:
            print(f"Warning: Plane detection found only {len(inliers)} inliers")
            # Use the whole point cloud if plane detection fails
            objects = pcd_down
        else:
            objects = pcd_down.select_by_index(inliers, invert=True)
        
        # Check if we have points to cluster
        if not objects.has_points() or len(objects.points) < 10:
            print("Warning: No objects remaining after plane removal")
            return []
        
        # Cluster objects
        labels = np.array(objects.cluster_dbscan(eps=0.02, min_points=50))
        num_clusters = labels.max() + 1
        
        if num_clusters <= 0:
            print("Warning: No clusters found")
            return []
        
        clusters = []
        for i in range(num_clusters):
            idx = np.where(labels == i)[0]
            if len(idx) > 0:
                clusters.append(objects.select_by_index(idx))
        
        print(f"Found {len(clusters)} clusters from {len(pcd.points)} points")
        return clusters
        
    except RuntimeError as e:
        print(f"Error during segmentation: {e}")
        return []

# ------------- NEW: Match YOLO → Open3D clusters -----------------

def match_yolo_to_clusters(cx, cy, depth_m, intr, clusters):
    yolo_X, yolo_Y, yolo_Z = pixel_to_point(intr, cx, cy, depth_m)
    if np.isnan(yolo_Z):
        return None, None

    det_point = np.array([yolo_X, yolo_Y, yolo_Z])
    min_dist = 999
    min_cluster = None

    for i, cluster in enumerate(clusters):
        if len(cluster.points) == 0:
            continue
        obb = cluster.get_oriented_bounding_box()
        center = np.array(obb.center)
        dist = np.linalg.norm(center - det_point)
        if dist < min_dist:
            min_dist = dist
            min_cluster = i

    return min_cluster, min_dist

# ------------------------ MAIN LOOP -----------------------------

def main():
    print(f"Loading YOLO model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)

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
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())

            # ---- YOLO detection ----
            results = model.predict(
                source=color_img,
                imgsz=IMG_SIZE,
                conf=CONF_THRESH,
                verbose=False
            )

            detections = []
            if len(results):
                for box in results[0].boxes:
                    xyxy = box.xyxy.cpu().numpy()[0]
                    conf = float(box.conf.cpu().numpy()[0])
                    cls_id = int(box.cls.cpu().numpy()[0])
                    cls_name = model.names[cls_id]
                    x1, y1, x2, y2 = map(int, xyxy)
                    detections.append((x1, y1, x2, y2, conf, cls_name))

            # ---- NEW: Open3D point cloud processing ----
            pcd = rs_frames_to_open3d(color_frame, depth_frame, intr)
            clusters = segment_objects_from_pcd(pcd)

            # Process each detection
            for (x1, y1, x2, y2, conf, cls_name) in detections:
                cx, cy = (x1+x2)//2, (y1+y2)//2
                depth_m = median_depth_in_bbox(depth_frame, cx, cy, patch=DEPTH_PATCH)

                text_extra = ""
                # Only try to match if we have clusters
                if len(clusters) > 0:
                    cluster_idx, dist = match_yolo_to_clusters(cx, cy, depth_m, intr, clusters)
                    
                    if cluster_idx is not None:
                        cluster = clusters[cluster_idx]
                        obb = cluster.get_oriented_bounding_box()
                        center = obb.center
                        size = obb.extent
                        text_extra = f" 3D: ({center[0]:.2f},{center[1]:.2f},{center[2]:.2f})m size({size[0]:.2f},{size[1]:.2f},{size[2]:.2f})"
                else:
                    # Fallback: just use YOLO depth
                    if not np.isnan(depth_m) and depth_m > 0:
                        X, Y, Z = pixel_to_point(intr, cx, cy, depth_m)
                        text_extra = f" ({X:.2f},{Y:.2f},{Z:.2f})m"

                label = f"{cls_name} {conf:.2f}{text_extra}"
                cv2.rectangle(color_img, (x1,y1), (x2,y2), (255,255,0), 2)
                cv2.putText(color_img, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

            # FPS display
            frame_count += 1
            if DISPLAY_FPS:
                now = time.time()
                if now - prev_time >= 1:
                    fps = frame_count / (now - prev_time)
                    prev_time = now
                    frame_count = 0
                    cv2.putText(color_img, f"FPS: {fps:.1f}", (10,20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            cv2.imshow("YOLO + Open3D RGB-D Pipeline", color_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
