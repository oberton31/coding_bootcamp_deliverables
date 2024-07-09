from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription({
        # Launch Talker node with custom parameters
        Node(
            package="lab1_pkg",
            executable="talker",
            name="talker",
            output="screen",
            parameters=[{"v": 1.0, "d": 2.0}]
        ),
        Node(
            package="lab1_pkg",
            executable="relay",
            name="relay",
            output="screen",
        )
    })