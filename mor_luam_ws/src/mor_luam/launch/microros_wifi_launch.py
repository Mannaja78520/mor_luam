from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    node_microros = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["udp4", "--port", "8888"],
    )
    
    ld.add_action(node_microros)
    return ld