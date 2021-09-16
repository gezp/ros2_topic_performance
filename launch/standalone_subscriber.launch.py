from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    name=LaunchConfiguration('name')
    enable_output_address=LaunchConfiguration('enable_output_address')
    enable_output_delay=LaunchConfiguration('enable_output_delay')

    declare_name = DeclareLaunchArgument(
        'name', default_value="subscriber")
    declare_enable_output_address = DeclareLaunchArgument(
        'enable_output_address', default_value="True")
    declare_enable_output_delay = DeclareLaunchArgument(
        'enable_output_delay', default_value="True")

    sub_node = Node(package='ros2_topic_performance',
        executable='standalone_subscriber',
        name = name,
        parameters=[
            {"enable_output_address":enable_output_address,
             "enable_output_delay":enable_output_delay},   
        ],
        output='screen')
    ld = LaunchDescription()
    ld.add_action(declare_name)
    ld.add_action(declare_enable_output_address)
    ld.add_action(declare_enable_output_delay)
    ld.add_action(sub_node)
    return ld