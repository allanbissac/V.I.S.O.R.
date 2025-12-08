#!/usr/bin/env python3

import time
import numpy as np
import cv2
import pyrealsense2 as rs
import open3d as o3d
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Point
from custom_interfaces.srv import GetNearestObject  
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
# manipulator_topic_name
# ---------------- USER CONFIG ----------------
MODEL_PATH = "/home/student10/Robotics_Systems_Design_Project/rgbd_vision/best.pt"
CONF_THRESH = 0.35
IMG_SIZE = 640
DEPTH_PATCH = 5
DISPLAY_FPS = True

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

def segment_objects_from_pcd(pcd):
    if not pcd.has_points() or len(pcd.points) < 10:
        return []
    
    pcd_down = pcd.voxel_down_sample(voxel_size=0.005)
    
    if len(pcd_down.points) < 3:
        return []
    
    try:
        plane_model, inliers = pcd_down.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=400
        )
        
        if len(inliers) < 100:
            objects = pcd_down
        else:
            objects = pcd_down.select_by_index(inliers, invert=True)
        
        if not objects.has_points() or len(objects.points) < 10:
            return []
        
        labels = np.array(objects.cluster_dbscan(eps=0.02, min_points=50))
        num_clusters = labels.max() + 1
        
        if num_clusters <= 0:
            return []
        
        clusters = []
        for i in range(num_clusters):
            idx = np.where(labels == i)[0]
            if len(idx) > 0:
                clusters.append(objects.select_by_index(idx))
        
        return clusters
        
    except RuntimeError as e:
        print(f"Error during segmentation: {e}")
        return []

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


class RGBDVisionNode(Node):
    def __init__(self):
        super().__init__('rgbd_vision_node')
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection3DArray, 
            '/object_detections_3d', 
            10
        )
        
        # TF broadcaster for camera frame
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store latest detections for service calls
        self.latest_detections = []
        self.detection_lock = False
        
        # Service to get nearest object
        self.nearest_obj_srv = self.create_service(
            Trigger,  # Using standard service, returns success and message
            'get_nearest_object',
            self.get_nearest_object_callback
        )
        
        # Publisher for nearest object (alternative to service)
        self.nearest_pub = self.create_publisher(
            PoseStamped,
            '/nearest_object_pose',
            10
        )
        
        self.get_logger().info('RGB-D Vision Node initialized')
        self.get_logger().info('Services: /get_nearest_object')
        self.get_logger().info('Topics: /object_detections_3d, /nearest_object_pose')

    def broadcast_camera_transform(self):
        """
        Broadcast the transform from robot base to camera.
        You'll need to measure and set these values based on your robot setup.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # or your robot's base frame
        t.child_frame_id = 'camera_link'
        
        # TODO: Set these based on your actual camera mounting position
        t.transform.translation.x = 0.0  # meters forward from base
        t.transform.translation.y = 0.0  # meters left from base
        t.transform.translation.z = 0.5  # meters up from base
        
        # Camera orientation (assuming camera points forward)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

    def get_nearest_object_callback(self, request, response):
        """
        Service callback to get the nearest detected object.
        Returns pose information as a string in the response.
        """
        if len(self.latest_detections) == 0:
            response.success = False
            response.message = "No objects detected"
            return response
        
        # Find nearest object (minimum Z distance in camera frame)
        nearest = min(self.latest_detections, key=lambda d: d[2][2])  # Sort by Z coordinate
        cls_name, conf, center, size = nearest
        
        # Calculate distance from camera
        distance = np.linalg.norm(center)
        
        response.success = True
        response.message = (
            f"class:{cls_name},"
            f"confidence:{conf:.2f},"
            f"x:{center[0]:.3f},"
            f"y:{center[1]:.3f},"
            f"z:{center[2]:.3f},"
            f"distance:{distance:.3f},"
            f"size_x:{size[0]:.3f},"
            f"size_y:{size[1]:.3f},"
            f"size_z:{size[2]:.3f}"
        )
        
        # Also publish as PoseStamped for easy visualization
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_link'
        pose_msg.pose.position.x = float(center[0])
        pose_msg.pose.position.y = float(center[1])
        pose_msg.pose.position.z = float(center[2])
        self.nearest_pub.publish(pose_msg)
        
        self.get_logger().info(f'Nearest object: {cls_name} at distance {distance:.3f}m')
        
        return response

    def publish_detections(self, detections_data):
        """
        Store and publish 3D detections.
        detections_data: list of tuples (cls_name, conf, center_xyz, size_xyz)
        """
        # Store for service calls
        self.latest_detections = detections_data
        
        # Broadcast camera transform
        self.broadcast_camera_transform()
        
        # Create Detection3DArray message
        det_array = Detection3DArray()
        det_array.header = Header()
        det_array.header.stamp = self.get_clock().now().to_msg()
        det_array.header.frame_id = 'camera_link'
        
        for (cls_name, conf, center, size) in detections_data:
            detection = Detection3D()
            
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = cls_name
            hypothesis.hypothesis.score = conf
            
            hypothesis.pose.pose.position.x = float(center[0])
            hypothesis.pose.pose.position.y = float(center[1])
            hypothesis.pose.pose.position.z = float(center[2])
            
            detection.results.append(hypothesis)
            
            detection.bbox.center.position.x = float(center[0])
            detection.bbox.center.position.y = float(center[1])
            detection.bbox.center.position.z = float(center[2])
            detection.bbox.size.x = float(size[0])
            detection.bbox.size.y = float(size[1])
            detection.bbox.size.z = float(size[2])
            
            det_array.detections.append(detection)
        
        self.detection_pub.publish(det_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = RGBDVisionNode()
    
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
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())

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

            pcd = rs_frames_to_open3d(color_frame, depth_frame, intr)
            clusters = segment_objects_from_pcd(pcd)

            detections_for_ros = []

            for (x1, y1, x2, y2, conf, cls_name) in detections:
                cx, cy = (x1+x2)//2, (y1+y2)//2
                depth_m = median_depth_in_bbox(depth_frame, cx, cy, patch=DEPTH_PATCH)

                text_extra = ""
                center = None
                size = None
                
                if len(clusters) > 0:
                    cluster_idx, dist = match_yolo_to_clusters(cx, cy, depth_m, intr, clusters)
                    
                    if cluster_idx is not None:
                        cluster = clusters[cluster_idx]
                        obb = cluster.get_oriented_bounding_box()
                        center = np.array(obb.center)
                        size = np.array(obb.extent)
                        
                        # Calculate distance from camera
                        distance = np.linalg.norm(center)
                        text_extra = f" D:{distance:.2f}m XYZ:({center[0]:.2f},{center[1]:.2f},{center[2]:.2f})"
                        
                        detections_for_ros.append((cls_name, conf, center, size))
                else:
                    if not np.isnan(depth_m) and depth_m > 0:
                        X, Y, Z = pixel_to_point(intr, cx, cy, depth_m)
                        center = np.array([X, Y, Z])
                        size = np.array([0.05, 0.05, 0.05])
                        distance = np.sqrt(X**2 + Y**2 + Z**2)
                        text_extra = f" D:{distance:.2f}m ({X:.2f},{Y:.2f},{Z:.2f})"
                        
                        detections_for_ros.append((cls_name, conf, center, size))

                label = f"{cls_name} {conf:.2f}{text_extra}"
                cv2.rectangle(color_img, (x1,y1), (x2,y2), (255,255,0), 2)
                cv2.putText(color_img, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

            if len(detections_for_ros) > 0:
                node.publish_detections(detections_for_ros)

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
