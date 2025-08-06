#!/usr/bin/env python3
import os
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ----------------------------
    # 1. Declare Launch Arguments
    # ----------------------------
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='/home/julian/aiina/src/mobile_robco_description/worlds/rgl_playground',
        # default_value='empty',
        description='Gazebo world file name (without extension)'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # ---------------------------------
    # 2. Get Package Share Directories
    # ---------------------------------
    robot_description_pkg = 'mobile_robco_description'
    robot_sim_pkg = 'mobile_robco_description'
    robot_description_path = get_package_share_directory(robot_description_pkg)
    robot_sim_path = get_package_share_directory(robot_sim_pkg)
    gazebo_pkg = 'ros_gz_sim' 

    # ---------------------------------
    # 3. Set Environment Variables
    # ---------------------------------
    # Existing GZ_SIM_RESOURCE_PATH for Gazebo worlds:
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(robot_sim_path, 'worlds'), ':',
            str(Path(robot_description_path).parent.resolve())
        ]
    )

    # Set Environment Variables for plugin paths and lidar patterns directory
    gz_sim_system_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            os.path.join(robot_description_path, 'RGL_Gazebo', 'RGLServerPlugin'), ':',
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ]
    )

    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=[
            os.path.join(robot_description_path, 'RGL_Gazebo', 'RGLVisualize'), ':',
            os.environ.get('GZ_GUI_PLUGIN_PATH', '')
        ]
    )

    rgl_patterns_dir = SetEnvironmentVariable(
        name='RGL_PATTERNS_DIR',
        value=os.path.join(robot_description_path, 'RGL_Gazebo', 'lidar_patterns')
    )

    # ---------------------------------
    # 4. Include Gazebo Launch File
    # ---------------------------------
    gazebo_launch_file = os.path.join(
        get_package_share_directory(gazebo_pkg),
        'launch',
        'gz_sim.launch.py'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'gz_args': [
                LaunchConfiguration('world'), '.sdf',
                ' -v 4',  # Verbosity level
                ' -r'     # Run headless if needed; remove if you want GUI
            ]
        }.items()
    )

    # ---------------------------------
    # 5. Process Xacro File into URDF
    # ---------------------------------
    xacro_file = os.path.join(robot_description_path, 'urdf', 'mobile_robco_gazebo.xacro')
    
    # Check if the Xacro file exists
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")
    
    # Process the Xacro file with 'use_sim' mapping
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    
    # ----------------------------
    # 6. Define Nodes
    # ----------------------------
    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )
    
    # Spawn Robot in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.4',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-name', 'mobile_robco',
            '-allow_renaming', 'false'
        ]
    )
    
    # Controller Loaders
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    load_robco_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'aiina_arm_controller'],
        output='screen'
    )

    load_lift_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'aiina_lift_controller'],
        output='screen'
    )

    load_arm_lift_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'aiina_lifted_arm_controller'],
        output='screen'
    )

    load_lifted_arm_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'aiina_lifted_arm_velocity_controller'],
        output='screen'
    )
    
    # Bridge Nodes (Expand as needed)
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_scan',
        output='screen',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan']
    )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_clock',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )

    # LiDAR Bridge Node
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_lidar',
        output='screen',
        arguments=['/sim_ouster/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked']
    )

    # cmd_vel Bridge Node
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_vel',
        output='screen',
        remappings=[('/cmd_vel', '/model/mobile_robco/cmd_vel')],
        arguments=['/model/mobile_robco/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']
    )

    # IMU Bridge Node
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_imu',
        output='screen',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU']
    )

    # Odom Bridge Node, remapped to /track_odom
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        output='screen',
        remappings=[('/model/mobile_robco/odometry', '/track_odom')],
        arguments=['/model/mobile_robco/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry']          
    )

    # FT Sensor Bridge Node
    bridge_ft_sensor = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ft_sensor',
        output='screen',
        arguments=['/ft_sensor@geometry_msgs/msg/WrenchStamped@gz.msgs.Wrench']
    )

    # Camera Bridge 
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # remappings=[('/world/rgl_playground/model/mobile_robco/link/C116_v1__1__1/sensor/camera_sensor/image', '/camera/image')],
        name='bridge_camera',
        arguments=['/world/rgl_playground/model/mobile_robco/link/C116_v1__1__1/sensor/camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    # Camera Info Bridge
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera_info',
        arguments=['/world/rgl_playground/model/mobile_robco/link/C116_v1__1__1/sensor/camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )

    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(robot_description_path, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
    )

    web_converter_node = Node(
        package='web_bridge',
        executable='web_converter',
        name='web_converter',
        output='screen'
    )
    
    # RViz2 Node
    # Path to the RViz config file in the `src` directory
    rviz_config_file = os.path.join(
        os.getcwd(), 
        'src',
        'mobile_robco_description',
        'config',
        'display.rviz'
    )
    
    # Check if RViz config file exists
    if not os.path.exists(rviz_config_file):
        raise FileNotFoundError(f"RViz config file not found: {rviz_config_file}")
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ---------------------------------
    # 7. Register Event Handlers
    # ---------------------------------
    # Ensure controllers are loaded after the robot is spawned
    controller_load_sequence = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_robco_velocity_controller]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_robco_velocity_controller,
                on_exit=[load_lift_velocity_controller]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_lift_velocity_controller,
                on_exit=[load_arm_lift_joint_trajectory_controller]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_lift_joint_trajectory_controller,
                on_exit=[load_lifted_arm_velocity_controller]
            )
        ),
    ]
    
    # ---------------------------------
    # 8. Assemble Launch Description
    # ---------------------------------
    ld = LaunchDescription()

    # Add Launch Arguments
    ld.add_action(world_arg)
    
    # Set Environment Variables for Gazebo resource, system plugin and GUI plugin paths
    ld.add_action(gazebo_resource_path)
    ld.add_action(gz_sim_system_plugin_path)
    ld.add_action(gz_gui_plugin_path)
    ld.add_action(rgl_patterns_dir)
    
    # Include Gazebo
    ld.add_action(gazebo)
    
    # Publish Robot State
    ld.add_action(node_robot_state_publisher)
    
    # Spawn Robot
    ld.add_action(gz_spawn_entity)
    
    # Register Event Handlers for Controller Loading
    for handler in controller_load_sequence:
        ld.add_action(handler)
    
    # Bridge Nodes
    ld.add_action(bridge_scan)
    ld.add_action(bridge_clock)
    ld.add_action(bridge_lidar)      # LiDAR bridge
    ld.add_action(bridge_cmd_vel)    # cmd_vel bridge
    ld.add_action(bridge_imu)        # IMU bridge
    ld.add_action(bridge_odom)       # Odom bridge
    ld.add_action(bridge_ft_sensor)  # FT Sensor bridge
    ld.add_action(bridge_camera)     # Camera bridge
    ld.add_action(bridge_camera_info)# Camera Info bridge

    ld.add_action(static_tf_pub)

    ld.add_action(robot_localization_node)

    # ld.add_action(web_socket_node)
    ld.add_action(web_converter_node)
    
    # Launch RViz
    ld.add_action(rviz)
    
    return ld
