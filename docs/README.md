# RO2 Iron Install

* Based on instructions: https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html
* Testing on Ubuntu 22.04 Jammy

## Installation

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-dev-tools

sudo apt update

sudo apt upgrade

sudo apt install ros-iron-desktop
```

## Setup Environment

```bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/iron/setup.bash# Replace ".bash" with your shell if you're not using bash

```

## Test Installation

```bash
# In one terminal:
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_cpp talker

# In another terminal:
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_py listener
```

## Install libfranka

```bash
mkdir -p ~/libs
cd ~/libs
sudo apt install -y libpoco-dev libeigen3-dev
git clone https://github.com/frankaemika/libfranka.git --recursive
cd libfranka
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF  ..
cmake --build . -j$(nproc)
cpack -G DEB
sudo dpkg -i libfranka-*.deb
```

## Install franka_ros2 dependencies

```bash
sudo apt install -y \
ros-iron-control-msgs \
ros-iron-xacro \
ros-iron-angles \
ros-iron-ros2-control \
ros-iron-realtime-tools \
ros-iron-control-toolbox \
ros-iron-moveit \
ros-iron-ros2-controllers \
ros-iron-joint-state-publisher \
ros-iron-joint-state-publisher-gui \
ros-iron-ament-cmake-clang-format \
python3-colcon-common-extensions
```

## Install franka_ros2 

```bash

mkdir -p ~/ros2_iron_ws/src                                                                     
cd ~/ros2_iron_ws/                                                                               

# Try the Humble version...
git clone https://github.com/mcbed/franka_ros2.git -b humble src/franka_ros2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release                                             

# Compiles OK!
```

## Test MoveIt2

```bash
# Run robot with fake HW and RVIZ2
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

```
[move_group-3] [ERROR] [1686918377.927480377] [moveit_trajectory_processing.time_optimal_trajectory_generation]: No acceleration limit was defined for joint panda_joint1! You have to define acceleration limits in the URDF or joint_limits.yaml
```

## Install MoveIt2 Tutorials

See full instructions [here](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html#create-a-colcon-workspace-and-download-tutorials) 

```bash
source /opt/ros/iron/setup.bash
sudo apt install python3-rosdep
sudo apt install python3-colcon-common-extensions 
sudo apt install python3-vcstool 

cd ~/ros2_iron_ws/
cd src
git clone https://github.com/ros-planning/moveit2_tutorials -b main --depth 1 
vcs import < moveit2_tutorials/moveit2_tutorials.repos
colcon build --mixin release --executor sequential
```

Build error:
```
--- stderr: rosparam_shortcuts                                                                            
CMake Error at CMakeLists.txt:97 (find_package): 
By not providing "Findros_testing.cmake" in CMAKE_MODULE_PATH this project has asked CMake to find a package configuration file provided by "ros_testing", but CMake did not find one.
```

## Install py_moveit

- not installing, requires MoveIt2 Tutorials to test properly
