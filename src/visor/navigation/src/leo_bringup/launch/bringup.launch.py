#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # ---- Launch arguments (tweak these from CLI or defaults) ----
    nav2_delay = LaunchConfiguration("nav2_delay")
    explore_delay = LaunchConfiguration("explore_delay")

    serial_port = LaunchConfiguration("serial_port")

    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")

    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")

    declare_args = [
        DeclareLaunchArgument("nav2_delay", default_value="5.0",
                              description="Seconds to wait before launching Nav2"),
        DeclareLaunchArgument("explore_delay", default_value="15.0",
                              description="Seconds to wait before launching Exploration (should be > nav2_delay)"),

        DeclareLaunchArgument("base_frame", default_value="base_link",
                              description="Parent TF frame"),
        DeclareLaunchArgument("laser_frame", default_value="laser",
                              description="Child TF frame (LaserScan frame_id)"),

        DeclareLaunchArgument("laser_x", default_value="0.0",
                              description="Laser x offset from base_link (m)"),
        DeclareLaunchArgument("laser_y", default_value="0.0",
                              description="Laser y offset from base_link (m)"),
        DeclareLaunchArgument("laser_z", default_value="0.0",
                              description="Laser z offset from base_link (m)"),

        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0",
                              description="Serial port for RPLiDAR"),
    ]

    # ---- 1) Start LiDAR launch ----
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rplidar_ros"),
                "launch",
                "rplidar_a2m12_launch.py"
            ])
        ),
        launch_arguments={
            "serial_port": serial_port
        }.items()
    )

    # ---- 2) Static TF: base_link -> laser (translation only, no rotation) ----
    # tf2_ros supports Euler as: x y z yaw pitch roll frame_id child_frame_id
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_static_tf",
        arguments=[
            laser_x, laser_y, laser_z,
            "0", "0", "0",          # yaw, pitch, roll = 0 (no rotation)
            base_frame, laser_frame
        ],
        output="screen",
    )

    # ---- 3) Start Nav2 launch (XML) with delay ----
    nav2_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("leo_nav"),
                "launch",
                "navigation.launch.xml"
            ])
        )
    )

    nav2_delayed = TimerAction(
        period=nav2_delay,
        actions=[nav2_launch]
    )

    # ---- 4) Start Exploration launch with delay ----
    exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("exploration"),
                "launch",
                "frontier_explorer.launch.py"
            ])
        )
    )

    exploration_delayed = TimerAction(
        period=explore_delay,
        actions=[exploration_launch]
    )

    return LaunchDescription(
        declare_args
        + [
            rplidar_launch,
            static_tf,
            nav2_delayed,
            exploration_delayed,
        ]
    )
