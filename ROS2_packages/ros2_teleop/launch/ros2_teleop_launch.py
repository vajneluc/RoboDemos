from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declaring parameters for launch
    start_joy = LaunchConfiguration('start_joy')

    return LaunchDescription([
        # Use pynput keyboard or sshkeyboard
        DeclareLaunchArgument(
            name='use_pynput',
            default_value='True'),
        # Start joy node when launched
        DeclareLaunchArgument(
            name='start_joy',
            default_value='True'),
        # Node controlling keyboard inputs
        Node(
            package='ros2_teleop',
            executable='keyboard_control',
            name='keyboard_control'
            ),
        # Node controlling controller inputs
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            condition=IfCondition(start_joy)
            ),
        # Node sending twist commands
        Node(
            package='ros2_teleop',
            executable='ros2_teleop_pymoveit2',
            name='ros2_teleop_pymoveit2'
        )
    ])