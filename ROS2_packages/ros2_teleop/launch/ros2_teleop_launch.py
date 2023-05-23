from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declaring parameters for launch
    start_joy = LaunchConfiguration('start_joy')
    use_meshcat = LaunchConfiguration('use_meshcat')
    use_pynput = LaunchConfiguration('use_pynput')
    topic_source = LaunchConfiguration('topic_source')

    return LaunchDescription([
        # Use pynput keyboard or sshkeyboard
        DeclareLaunchArgument(
            name='use_pynput',
            default_value='True'),
        # Start joy node when launched
        DeclareLaunchArgument(
            name='start_joy',
            default_value='True'),
        # Use meshcat visualization
        DeclareLaunchArgument(
            name='use_meshcat',
            default_value='False'),
        # Chose topic source [/joint_states or /mytopic]
        DeclareLaunchArgument(
            name='topic_source',
            default_value='/joint_states'),
        # Node controlling keyboard inputs
        Node(
            package='ros2_teleop',
            executable='keyboard_control',
            name='keyboard_control',
            parameters=[{'use_pynput':use_pynput}]
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
        ),
        # Node visualizing in meshcat
        Node(
            package='meshcat_visualizer',
            executable='meshcat_visualizer_node',
            name='meshcat_visualizer_node',
            condition=IfCondition(use_meshcat),
            parameters=[{'topic_source': topic_source}]
        )
    ])