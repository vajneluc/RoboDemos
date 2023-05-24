from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_msg = f"...PRESS ENTER TO CONTINUE...\n\n\nType h for help"
    print(launch_msg)
    config = input()
    if config == "h":
        FORMAT_BOLD = "\033[1m"
        FORMAT_ITALIC = "\033[3m"
        FORMAT_UNDERLINE = "\033[4m"
        FORMAT_RESET = "\033[0m"
        RED = "\033[31m"
        GREEN = "\033[32m"
        BLUE = "\033[34m"
        config_msg = (
            f"\n\n\n{FORMAT_BOLD}{FORMAT_ITALIC}{FORMAT_UNDERLINE}AVAILABLE LAUNCH CONFIGURATIONS :{FORMAT_RESET}\n\n\n"
            f"{BLUE}start_joy:={FORMAT_RESET}[{GREEN}true{FORMAT_RESET} ({FORMAT_UNDERLINE}default{FORMAT_RESET}) / {RED}false{FORMAT_RESET}]\n"
            f"{BLUE}use_pynput:={FORMAT_RESET}[{GREEN}true{FORMAT_RESET} ({FORMAT_UNDERLINE}default{FORMAT_RESET}) / {RED}false{FORMAT_RESET}]\n"
            f"{BLUE}use_meshcat:={FORMAT_RESET}[{GREEN}true{FORMAT_RESET} / {RED}false{FORMAT_RESET} ({FORMAT_UNDERLINE}default{FORMAT_RESET})]\n"
            f"{BLUE}topic_source:={FORMAT_RESET}[{GREEN}/my_topic{FORMAT_RESET} ({FORMAT_UNDERLINE}default:{FORMAT_RESET} {GREEN}/joint_states{FORMAT_RESET})\n\n\n]"
        )
        print(config_msg)

    # Declaring parameters for launch
    start_joy = LaunchConfiguration("start_joy")
    use_meshcat = LaunchConfiguration("use_meshcat")
    use_pynput = LaunchConfiguration("use_pynput")
    topic_source = LaunchConfiguration("topic_source")

    return LaunchDescription(
        [
            # Use pynput keyboard or sshkeyboard
            DeclareLaunchArgument(name="use_pynput", default_value="True"),
            # Start joy node when launched
            DeclareLaunchArgument(name="start_joy", default_value="True"),
            # Use meshcat visualization
            DeclareLaunchArgument(name="use_meshcat", default_value="False"),
            # Chose topic source [/joint_states or /mytopic]
            DeclareLaunchArgument(name="topic_source", default_value="/joint_states"),
            # Node controlling keyboard inputs
            Node(
                package="ros2_teleop",
                executable="keyboard_control",
                name="keyboard_control",
                prefix="xterm -e",
                parameters=[{"use_pynput": use_pynput}],
            ),
            # Node controlling controller inputs
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                condition=IfCondition(start_joy),
            ),
            # Node sending twist commands
            Node(
                package="ros2_teleop",
                executable="ros2_teleop_pymoveit2",
                name="ros2_teleop_pymoveit2",
                parameters=[{"start_joy": start_joy}],
            ),
            # Node visualizing in meshcat
            Node(
                package="meshcat_visualizer",
                executable="meshcat_visualizer_node",
                name="meshcat_visualizer_node",
                condition=IfCondition(use_meshcat),
                parameters=[{"topic_source": topic_source}],
            ),
        ]
    )
