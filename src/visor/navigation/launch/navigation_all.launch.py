# It's still a draft. We should modify it by our real information.
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # -- 1) Static TF: Example base_link -> laser, 90 degrees around the Z-axis --
    # Note: The parameters of static_transform_publisher are x y z yaw pitch roll parent child (in some versions, it is roll pitch yaw).
    # Here is a common way to write it: yaw=1.57079632679 (90deg)
    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_laser",
        arguments=["0.075", "0.0", "0.0", "0.0", "0.0", "1.57079632679", "base_link", "laser"],
        output="screen"
    )

    # If you still have static TFS such as cameras or IMUs, just copy another Node
    # static_tf_cam = Node(...)

    # --- 2) include your navigation launch (XML) --
    leo_nav_share = get_package_share_directory("leo_nav")
    nav_launch_path = os.path.join(leo_nav_share, "launch", "navigation.launch.xml")

    leo_navigation = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(nav_launch_path)
    )

    # --- 3) our explore node ---
    explore_node = Node(
        package="my_explore_pkg",
        executable="explore_node",
        name="explore",
        output="screen",
        parameters=[
            # You can add parameter files here, for example:
            # os.path.join(get_package_share_directory("my_explore_pkg"), "config", "explore.yaml")
        ],
        remappings=[
            # If remapping is required，such as /cmd_vel -> /nav/cmd_vel
            # ("/cmd_vel", "/cmd_vel"),
        ]
    )

    return LaunchDescription([
        static_tf_laser,
        leo_navigation,
        explore_node
    ])