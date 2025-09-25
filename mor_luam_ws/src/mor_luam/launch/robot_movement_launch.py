from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    default_params = os.path.join(
        FindPackageShare('mor_luam').perform(None),
        'config', 'robot_movement.yaml'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to robot movement params'
    )

    node = Node(
        package='mor_luam',
        executable='robot_movement.py',
        name='robot_movement_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_arg, node])
