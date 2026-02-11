from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # Arena bounds (adjust if your map origin differs)
        DeclareLaunchArgument("min_x", default_value="-0.55"),
        DeclareLaunchArgument("max_x", default_value="5.35"),
        DeclareLaunchArgument("min_y", default_value="-0.55"),
        DeclareLaunchArgument("max_y", default_value="5.35"),

        Node(
            package="exploration",
            executable="frontier_explorer",
            name="frontier_explorer",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"min_x": LaunchConfiguration("min_x")},
                {"max_x": LaunchConfiguration("max_x")},
                {"min_y": LaunchConfiguration("min_y")},
                {"max_y": LaunchConfiguration("max_y")},
            ],
        ),
    ])
