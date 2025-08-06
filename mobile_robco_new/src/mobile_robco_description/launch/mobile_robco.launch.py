#!/usr/bin/env python3
import os
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_description_path = get_package_share_directory('mobile_robco_description')

    xacro_file = os.path.join(robot_description_path, 'urdf', 'mobile_robco.xacro')
    
    # Check if the Xacro file exists
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")
    
    # Process the Xacro file with 'use_sim' mapping
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    
    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, os.path.join(robot_description_path, 'config', 'controller_config.yaml')],
        output='screen'
    )

    robot_track_odometry_node = Node(
        package='mobile_robco_description',
        executable='track_odom_publisher_node',
        name='track_odom_publisher_node',
        output='screen',
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
        parameters=[os.path.join(robot_description_path, 'config/ekf.yaml')]
    )

    robco_joint_angle_observer = Node(
        package='robco_interface',
        executable='robco_joint_angle_observer',
        output='screen'
    )

    multi_camera_launch_file = os.path.join(
        get_package_share_directory('camera360'),
        'launch',
        'multi_camera.launch.py'
    )

    lidar_launch_file = os.path.join(
        get_package_share_directory('ouster_ros'),
        'launch',
        'driver.launch.py'
    )
        
    # Controller Loaders

    load_aiina_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'aiina_controller'],
        output='screen'
    )

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

    load_lifted_arm_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'aiina_lifted_arm_velocity_controller'],
        output='screen'
    )

        
    rviz_config_file = os.path.join(
        os.getcwd(), 
        'src',
        'mobile_robco_description',
        'config',
        'aiina.rviz'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        # arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    foxglove_websocket_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml'
    )
            
    # ---------------------------------
    # 8. Assemble Launch Description
    # ---------------------------------
    ld = LaunchDescription()
    
    # Publish Robot State
    ld.add_action(node_robot_state_publisher)

    # Start Controller Manager
    ld.add_action(ros2_control_node)

    # Publish Track Odometry
    ld.add_action(robot_track_odometry_node)

    # Publish Static Transform
    # ld.add_action(static_tf_pub)

    # Robot Localization
    ld.add_action(robot_localization_node)

    # Start Joint Angle Observer
    ld.add_action(robco_joint_angle_observer)

    # Load Camera
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'launch', multi_camera_launch_file],
        output='screen'
    ))

    # Load Lidar
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'launch', lidar_launch_file],
        output='screen'
    ))

    # Load Controllers
    # ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_aiina_controller)
    ld.add_action(load_robco_velocity_controller)
    ld.add_action(load_lift_velocity_controller)
    ld.add_action(load_lifted_arm_velocity_controller)
        
    # Launch RViz
    ld.add_action(rviz)

    # Load Foxglove Bridge
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'launch', foxglove_websocket_launch],
        output='screen'
    ))
    
    return ld
