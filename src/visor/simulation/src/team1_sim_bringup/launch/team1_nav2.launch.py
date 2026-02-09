#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_team1 = get_package_share_directory("team1_sim_bringup")
    pkg_nav2 = get_package_share_directory("leo_nav")

    sim_launch = os.path.join(pkg_team1, "launch", "team1_leo_sim.launch.py")
    nav2_bringup = os.path.join(pkg_nav2, "launch", "navigation.launch.xml")
    nav2_rviz = os.path.join(pkg_nav2, "launch", "rviz.launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_delay = LaunchConfiguration("nav2_delay")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("nav2_delay", default_value="5.0"),

        # 1) Your Gazebo + spawn
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items()
        ),

        # # 2) Nav2 bringup
        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(nav2_bringup),
        #     launch_arguments={
        #         "use_sim_time": LaunchConfiguration("use_sim_time"),
        #     }.items()
        # ),

        # # 3) RViz pre-wired for namespacing (/tf,/map remaps)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(nav2_rviz),
        #     launch_arguments={
        #         "use_sim_time": LaunchConfiguration("use_sim_time"),
        #     }.items()
        # ),

        # 2) Nav2 + RViz AFTER delay
        TimerAction(
            period=nav2_delay,
            actions=[
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(nav2_bringup),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                    }.items()
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_rviz),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                    }.items()
                ),
            ]
        ),
    ])
