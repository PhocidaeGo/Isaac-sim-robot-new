# Issac-sim

## nvidia_isaac-sim_ros2_docker
### Prerequisites
- NVIDIA Drivers installation: https://ubuntu.com/server/docs/nvidia-drivers-installation<br>
GPU drivers version must be 535.129.03 or later, check it with:
```bash
nvidia-smi
```

- Docker installation and executing without sudo:
```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```
```bash
# Post-install steps for Docker
sudo groupadd docker # Create group
sudo usermod -aG docker $USER # Add current user to docker group
newgrp docker # Log in docker group
```
```bash
#Verify Docker installation
docker run hello-world
```

- NVIDIA Container Toolkit installation:
```bash
# Configure the repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
  && \
    sudo apt-get update

# Install the NVIDIA Container Toolkit packages
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Configure the container runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Verify NVIDIA Container Toolkit
docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

- Generate NGC API Key: https://docs.nvidia.com/ngc/ngc-overview/index.html#generating-api-key
- Log in to NGC:
```bash
docker login nvcr.io
```
```bash
Username: $oauthtoken
Password: <Your NGC API Key>
WARNING! Your password will be stored unencrypted in /home/username/.docker/config.json.
Configure a credential helper to remove this warning. See
credentials-store
Login Succeeded
```

### Build image
```bash
docker build -t {IMAGE_NAME}:{TAG} .
```
Example:
```bash
docker build -t isaac_sim_ros2:4.2.0-Humble .
```

### Run container
Allow running graphic interfaces in the container:
```bash
xhost +
```
Run the container with the needed configuration:
```bash
docker run --name isaac-moveit --privileged --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --network=host   -e "PRIVACY_CONSENT=Y"   -v $HOME/.Xauthority:/root/.Xauthority   -e DISPLAY   -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw   -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw   -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw   -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw   -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw   -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw   -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw   -v ~/docker/isaac-sim/documents:/root/Documents:rw   -v /home/student/Documents/robco:/isaac-sim/robco   isaac_sim_ros2:4.2.0-Humble
```

### Run Isaac Sim inside the container
If this command is included when running the container, ROS2 bridge will fail. That's because the container with ROS2 packages must be started first, and then Isaac Sim.
Once the container is running, type next line in the container:
```bash
./runapp.sh
```
Wait until Isaac Sim is completely loaded. Ignore "not responding" messages, it will take some time, so be patient.

## Build package and set Moveit
1. Install all needed packages inside container:
```bash
apt update
apt install ros-humble-robot-localization
apt install ros-humble-ros2-control
apt install ros-humble-ros2-controllers
apt install ros-humble-gripper-controllers
apt install ros-humble-moveit
apt install ros-humble-topic-based-ros2-control
```

2. Build the package and source:
```bash
cd robco
colcon build
source install/setup.bash
```

3. Config the robot arm:
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
In the GUI, following the tutorial: https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html

After generating the packge, check if the moveit_controllers.yaml has following lines:
```bash
  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - robco_joint_0
      - robco_joint_1
      - robco_joint_2
      - robco_joint_3
      - robco_joint_4
      - robco_joint_5
    action_ns: follow_joint_trajectory
    default: true
```
Build the packages once evenything is done.

4. Check if Moveit works:
```bash
cd robco
source install/setup.bash
ros2 launch robco_moveit_config demo.launch.py
```
Plan and execute a movement, see if the terminal shows "Execute request success!". If yes, proceed to next step.

5. Convet .xacro file to .urdf file for Issac.
```bash
cd robco
source install/setup.bash
cd src/robco_moveit_config/config
xacro mobile_robco.urdf.xacro > mobile_robco.urdf
```

## Start Isaac Sim
1. Import the model into Isaac
Open Isaac-sim, import this model. Check if the articulation root is under base_link. Then connect the action graph as following:
![alt text](media/moveit_action_graph.png)

Start the simulation and Moveit, now you can see the robot in isaac moves along with the one in RViz.

2. Pubulish following command to ros topic, you should see the robot is moving, if not, check the articulation root.
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{'linear': {'x': 0.2, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}"
```

## Simulating People in Isaac Sim
Follow the tutorial: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/replicator_tutorials/ext_replicator-agent/ext_omni_anim_people.html

Overall steps:
1. Enable Isaacsim.Replicator.Agent
2. In 'Agent SDG' -> 'Global Settings', set the 'Scene Asset Path' to desired scene.
3. Creating the NavMesh for characters: https://docs.omniverse.nvidia.com/extensions/latest/ext_navigation-mesh.html, check the 'NavMesh'->'Geometry', there is no object in the 'Excluded Geometry' and the 'Auto-Exclude Rigid Bodies' are disabled.
4. Cofig other settings
5. Click 'Set Up Simulation' in 'Global Settings'. Start Simulation.
http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Environments/Simple_Warehouse/full_warehouse.usd

## Using ZED camera in Isaac
Follow the tutorial: https://www.stereolabs.com/docs/isaac-sim/ros2_integration
### Inside a container:
#### 1. Install CUDA
1. Check nvidia-smi
2. Install CUDA Toolkit (Ubuntu 22.04 & CUDA 12.4)
```bash
apt-get update && apt-get install -y wget gnupg
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600

wget https://developer.download.nvidia.com/compute/cuda/12.4.1/local_installers/cuda-repo-ubuntu2204-12-4-local_12.4.1-550.54.15-1_amd64.deb
dpkg -i cuda-repo-ubuntu2204-12-4-local_12.4.1-550.54.15-1_amd64.deb
cp /var/cuda-repo-ubuntu2204-12-4-local/cuda-*-keyring.gpg /usr/share/keyrings/

apt-get update
```
```bash
apt-get -y install cuda-toolkit-12-4
```
Add CUDA to PATH:
```bash
export PATH=/usr/local/cuda-12.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64:$LD_LIBRARY_PATH
```
3. Check installtion:
```bash
nvcc --version
nvidia-smi
```

#### 2. Install ZED SDK:
https://www.stereolabs.com/docs/installation/linux
Use the '-- silent' command

#### 3. Install and launch ROS2 Wrapper, check the output in RViz

### On host:
#### 1. Update nvidia driver
```bash
sudo apt update
sudo apt install ubuntu-drivers-common
sudo ubuntu-drivers autoinstall
sudo reboot
```
Check if the cuda version matches the installation file (reconmand for host: ZED_SDK_Ubuntu24_cuda12.8_tensorrt10.9_v5.0.1.zstd.run)

#### 2. Install ZED SDK:
https://www.stereolabs.com/docs/installation/linux

## ZED API:
https://www.stereolabs.com/docs/app-development/python/install
If ZED api has problem to launch, check https://support.stereolabs.com/hc/en-us/articles/8422008229143-How-can-I-solve-ZED-SDK-OpenGL-issues-under-Ubuntu

### Build mesh:
```bash
cd /usr/local/zed/samples/'spatial mapping'/'spatial mapping'/python
python3 spatial_mapping.py --build_mesh
```
Alternative: Using ZUDfu

### Convert .obj to .usd file for Isaac

### ZED ROS2 Wrapper:
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i svo_path:=/isaac-sim/robco/zed/test.svo2
```

## Useful commands
import omni.replicator.core as rep

camera = rep.create.camera()

with rep.trigger.on_frame():
    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform((1800, 1000, -1400), (1800, 2000, 1900)),
            look_at="/root/Cube",
        )

```bash
docker exec -it isaac-moveit bash
```

## Tips:
1. Kill a process in container:
``` bash
# Find PID
docker exec isaac-moveit ps aux
# Kill
docker exec isaac-moveit kill -9 <pid>
```

2. File acess:
```bash
sudo chown -R $USER:$USER ~/Documents/robco
```

3. If you see the following error:
```bash
nvidia-smi
NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver. Make sure that the latest NVIDIA driver is installed and running
```
do following:
```bash
# Purge existing drivers
sudo apt-get purge '^nvidia-.*'
sudo apt-get autoremove

# Add the official graphics-drivers PPA (optional, but recommended)
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Find recommended driver version
ubuntu-drivers devices

# Install the recommended driver
(example) sudo apt install nvidia-driver-575-open

sudo reboot
```


# Isaac Sim SGD:
### 1. Add semantic label to objects
When use 'Synthetic Data Recorder', if you want to visualized the saved segmented image, make sure you checked BOTH 'semantic_segmentation' and 'colorize_semantic_segmentation'.

The recording can be excuted even if simulation is not started.
### 2. Randomazation:
in script editor, run the code. Note: make sure the range is ordered
https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/randomizer_details.html#randomizing-dome-lights