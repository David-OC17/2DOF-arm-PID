import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["gnome-terminal", "--", "ros2", "run", "ui_controller", "ui_node"],
            output="screen"
        ),
        
        ExecuteProcess(
            cmd=["gnome-terminal", "--", "ros2", "run", "inverse_kinematics", "inverse_kinematics_server"],
            output="screen"
        ),

    ])