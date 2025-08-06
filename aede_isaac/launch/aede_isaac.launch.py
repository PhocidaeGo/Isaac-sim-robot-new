from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    # Path to AEDE's internal launch file
    aede_launch_file_path = os.path.join(
        os.getenv('HOME'),
        '/isaac-sim/robco/aede_isaac/src/autonomous_exploration_development_environment/src/vehicle_simulator/launch/system_real_robot.launch'
    )

    return LaunchDescription([
        # 1. Transform cloud to map frame
        Node(
            package='cloud_to_map',
            executable='cloud_to_map',
            name='cloud_to_map'
        ),

        # 2. Transform odom to map frame
        Node(
            package='odom_to_map',
            executable='odom_transform_node',
            name='odom_transform_node'
        ),

        # 3. Launch AEDE full system
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(aede_launch_file_path)
        ),

        # 4. Twist converter for Isaac-Sim
        Node(
            package='twist_converter_pkg',
            executable='twist_converter',
            name='twist_converter'
        ),
    ])
