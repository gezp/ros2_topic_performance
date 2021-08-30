import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    composition_node = Node(package='ros2_topic_performance',
        executable='composition_1pub_1sub',
        parameters=[
            {"rate" : 50},
            {"message_length":100000},
            {"use_unique_ptr":True}
        ],
        output='screen')
    ld.add_action(composition_node)
    return ld