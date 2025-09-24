# AEDE in Isaac Sim
## Installation
### 1. Install the AEDE:
```bash
sudo apt update
sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

```bash
sudo apt update
sudo apt install libusb-dev ros-humble-desktop-full ros-humble-joy ros-humble-gazebo-msgs \
ros-humble-gazebo-plugins ros-humble-gazebo-ros ros-humble-gazebo-ros2-control \
ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.sh
```

Setup: https://drive.google.com/file/d/1jW1jFDvRsUWcfivC6WWcHHdX-JDSzPGo/view

### 2. Use ZED camera in Isaac-sim with ROS2
https://www.stereolabs.com/docs/isaac-sim/ros2_integration

### 3. Install TARE
Follow the instruction in the repo.

## Usage
```bash
# Launch ZED
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx sim_mode:=true use_sim_time:=true

# Transform cloud to map frame
cd robco/aede/cloud_to_map
source install/setup.bash
ros2 run cloud_to_map cloud_to_map

# Transform odom to map frame
cd robco/aede/odom_to_map
source install/setup.bash
ros2 run odom_to_map odom_transform_node

# Launch AEDE
cd robco/aede/autonomous_exploration_development_environment
source install/setup.bash
ros2 launch vehicle_simulator system_real_robot.launch

# Twist messages: cmd_vel for Isaac-Sim 
cd robco/aede/twist_converter_pkg
source install/setup.bash
ros2 run twist_converter_pkg twist_converter

# Launch TARE
cd robco/aede/tare_planner
source install/setup.sh
ros2 launch tare_planner explore_garage.launch

# Save images
cd robco/aede/twist_converter_pkg
source install/setup.bash
ros2 run twist_converter_pkg save_image

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/Twist "{'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}"
```

```bash
cd robco/aede_isaac
source install/setup.bash
ros2 launch launch/aede_isaac.launch.py
```
