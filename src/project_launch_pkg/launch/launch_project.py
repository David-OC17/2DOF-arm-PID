import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the ui_controller node (from the ui_controller package)
        Node(
            package='ui_controller',         # Package name
            executable='ui_node',             # Executable name
            name='ui_node',                   # Node name
            output='screen',                  # Output to screen
        ),
        # Launch the inverse_kinematics_server node (from the inverse_kinematics package)
        Node(
            package='inverse_kinematics',    # Package name
            executable='inverse_kinematics_server',  # Executable name
            name='inverse_kinematics_server',  # Node name
            output='screen',                 # Output to screen
        ),
    ])
