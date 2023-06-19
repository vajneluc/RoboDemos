# ROS2 Humble Moveit2

## Check for latest versions of packages installed

```bash
sudo apt update
sudo apt dist-upgrade
rosdep update
```

## Source installation requires various ROS2 build tools

```bash
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget && \
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
  ```

## Uninstall Any Pre-existing MoveIt Debians

```bash
sudo apt remove ros-humble-moveit*
```

## Download Source Code

```bash
git clone https://github.com/ros-planning/moveit2.git -b humble
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_humble.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
```

## Build MoveIt

```bash
cd ~/ros2_ws
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Source the Workspace

```bash
source setup/install/setup.bash
```