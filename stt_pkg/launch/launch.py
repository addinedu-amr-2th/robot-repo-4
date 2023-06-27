from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="stt_pkg",
            executable="gpt_agent",
            output="screen"
        ),
        Node(
            package="schedule_maker",
            executable="schedule_maker",
            output="screen"
        ),
    ])