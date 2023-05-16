# ROS2 Keyboard and Controller Teleop
## Description
This ROS2 teleop can be used to control a robot visualization in rviz2 using a keyboard and a controller.
## Required packages
Pymoveit2 package
```bash
git clone https://github.com/AndrejOrsula/pymoveit2.git
```
Joy package
```bash
sudo apt install ros-humble-joy
```
Keyboard State package
```bash
git clone https://github.com/JuliusTomsa/RoboDemos/tree/2a6d06fd570404dc02fea6478fa6065a3a82b477/ROS2_packages/keyboard_msgs
```
ROS2 Teleop package
```bash
git clone https://github.com/JuliusTomsa/RoboDemos/tree/2a6d06fd570404dc02fea6478fa6065a3a82b477/ROS2_packages/ros2_teleop
```
## How to
1. Build
```bash
colcon build --packages-select pymoveit2 keyboard_msgs ros2_teleop 
```
2. Run the Keyboard and Controller nodes
```bash
ros2 run ros2_teleop keyboard_control
ros2 run joy joy_node
```
3. Launch Rviz2 configuration
```bash
ros2 launch panda_moveit_config ex_fake_control.launch.py
```
4. Run the Teleop node
```bash
# Start the servo
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
# Run the node
ros2 run ros2_teleop ros2_teleop_pymoveit2
```
## Keybindings
### Controller
#### Move
* R_JOYSTICK (X + Y Axis)
* L_JOYSTICK_UP/DOWN (Z Axis)
#### Turn
* ARROW BUTTONS (X + Y Axis)
* L_JOYSTICK_LEFT/RIGHT (Z Axis)
#### Speed
* I K (Linear speed)
* O L (Angular speed)
### Keyboard
#### Move
* W A S D (X + Y Axis)
* SHIFT or CTRL (Z Axis)
#### Turn
* ARROW KEYS (X + Y Axis)
* Q E (Z Axis)
#### Speed
* BUTTON4 BUTTON1 (Linear speed)
* BUTTON2 BUTTON3 (Angular speed)

