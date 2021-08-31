import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    pub_node= Node(package='ros2_topic_performance',
        executable='standalone_publisher',
        parameters=[
            {"rate" : 50},
            {"message_length":100000},
            {"use_unique_ptr":False}
        ],
        output='screen')
    ld.add_action(pub_node)
    sub_node = Node(package='ros2_topic_performance',
        executable='standalone_subscriber',
        output='screen')
    ld.add_action(sub_node)
    return ld