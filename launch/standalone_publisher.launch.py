from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    name=LaunchConfiguration('name')
    rate=LaunchConfiguration('rate')
    point_num=LaunchConfiguration('point_num')
    use_unique_ptr=LaunchConfiguration('use_unique_ptr')
    enable_output_address=LaunchConfiguration('enable_output_address')

    declare_name = DeclareLaunchArgument(
        'name', default_value="publisher")
    declare_rate = DeclareLaunchArgument('rate', default_value="1")
    declare_point_num = DeclareLaunchArgument(
        'point_num', default_value="1000")
    declare_use_unique_ptr = DeclareLaunchArgument(
        'use_unique_ptr', default_value="False")
    declare_enable_output_address = DeclareLaunchArgument(
        'enable_output_address', default_value="True")

    pub_node= Node(package='ros2_topic_performance',
        executable='standalone_publisher',
        name = name,
        parameters=[
            {"rate" : rate,
             "point_num": point_num,
             "use_unique_ptr": use_unique_ptr,
             "enable_output_address":enable_output_address}, 
        ],
        output='screen')
    ld = LaunchDescription()
    ld.add_action(declare_name)
    ld.add_action(declare_rate)
    ld.add_action(declare_point_num)
    ld.add_action(declare_use_unique_ptr)
    ld.add_action(declare_enable_output_address)
    ld.add_action(pub_node)
    return ld