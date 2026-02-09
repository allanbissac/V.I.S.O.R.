#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Defaults (portable, package-relative) ---
    team1_worlds_share = get_package_share_directory("team1_gz_worlds")
    default_world_path = os.path.join(team1_worlds_share, "worlds", "team1_arena.sdf")

    team1_bringup_share = get_package_share_directory("team1_sim_bringup")
    world_only_launch = os.path.join(team1_bringup_share, "launch", "leo_gz_world_only_team1.launch.py")
    spawn_launch = os.path.join(team1_bringup_share, "launch", "spawn_robot_team1.launch.py")

    # --- Ensure Gazebo can resolve any models/world assets from your package ---
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    team1_desc_share = get_package_share_directory("team1_leo_description")

    new_gz_path = existing_gz_path + (os.pathsep if existing_gz_path else "") + team1_worlds_share + os.pathsep + team1_desc_share

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ns",
            default_value="",
            description="Robot namespace (e.g., leo1)"
        ),
        DeclareLaunchArgument(
            "sim_world",
            default_value=default_world_path,
            description="Absolute path to the SDF world file"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="True"),

        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=new_gz_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_only_launch),
            launch_arguments={ 
                "sim_world": LaunchConfiguration("sim_world") 
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={ 
                "robot_ns": LaunchConfiguration("robot_ns") 
            }.items()
        ),

        # ---- Bridges (single source of truth for all topics) ----
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="team1_parameter_bridge",
            output="screen",
            arguments=[
                # ROS -> GZ
                ["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],

                # GZ -> ROS
                ["/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
                # ["/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
                ["/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU"],
                ["/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model"],

                # Existing camera info (if you still use it)
                ["/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],

                # NEW: LiDAR
                ["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],

                # NEW: RGBD camera
                # ["/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
                # ["/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],

                # # OPTIONAL: scan point cloud (you have /leo1/scan/points)
                # ["/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
            ],
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"qos_overrides./tf_static.publisher.durability": "transient_local"},
            ],
        ),

        Node(
            package="ros_gz_image",
            executable="image_bridge",
            name="team1_image_bridge",
            output="screen",
            arguments=[
                # Existing camera image
                ["/camera/image_raw"],

                # NEW: RGBD images
                ["/rgbd_camera/image"],
                ["/rgbd_camera/depth_image"],
            ],
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"qos": "sensor_data"}
            ],
        ),

        Node(
            package="team1_sim_bringup",
            executable="odom_tf_broadcaster",
            name="odom_tf_broadcaster",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        )

    ])
