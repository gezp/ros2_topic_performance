import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    pub1_node= Node(package='ros2_topic_performance',
        executable='standalone_publisher',
        parameters=[
            {"rate" : 50},
            {"message_length":100000},
            {"use_unique_ptr":True}
        ],
        output='screen')
    ld.add_action(pub1_node)
    sub1_node = Node(package='ros2_topic_performance',
        executable='standalone_subscriber',
        name = 'subscriber1',
        output='screen')
    ld.add_action(sub1_node)
    sub2_node = Node(package='ros2_topic_performance',
        executable='standalone_subscriber',
        name = 'subscriber2',
        output='screen')
    ld.add_action(sub2_node)
    return ld