0. Pull the repos using vcs tool
```bash
export PATH=$HOME/.local/bin:$PATH
source ~/.bashrc  # or: source ~/.zshrc
vcs import < ~/Documents/Production/docker/volumes/ros2_ws/src/aiina/src/aiina.repos
```

1. Run the container
```bash
cd /home/yuanyan/Documents/Production/docker
make shell
source install/setup.bash
```

2. Start all services
In order to let this container (ROS2 Jazzy) communicate with the Isaac container (ROS2 Humble), run following in each new-opened terminal in the two container at first:
```bash
# Install if you have not
sudo apt update
sudo apt install ros-humble/jazzy-rmw-cyclonedds-cpp
# Use Cyclone DDS
# Humble: Fast DDS (rmw_fastrtps_cpp) is the default.
# Jazzy: CycloneDDS (rmw_cyclonedds_cpp) is the default.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
```
Then,
```bash
start
```

3. Choose a window group to view:
```bash
byobu
```
Then enter the number of interested window

4. Press F3 or F4 to switch to another window, press F2 to create a new window, press F6 to exit current window group.

TODO: 

topics:
joint commands: /aiina_lifted_arm_velocity_controller/commands
joint states: (?) joint_states
image: /front/camera/rgb/image_rect_color/compressed