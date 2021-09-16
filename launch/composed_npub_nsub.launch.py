from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # for container
    publisher_num = LaunchConfiguration('publisher_num')
    subscriber_num = LaunchConfiguration('subscriber_num')
    use_intra_process_comms = LaunchConfiguration('use_intra_process_comms')
    use_dedicated_executors = LaunchConfiguration('use_dedicated_executors')

    # for node
    rate=LaunchConfiguration('rate')
    point_num=LaunchConfiguration('point_num')
    use_unique_ptr=LaunchConfiguration('use_unique_ptr')
    enable_output_address=LaunchConfiguration('enable_output_address')
    enable_output_delay=LaunchConfiguration('enable_output_delay')

    # declare
    declare_publisher_num = DeclareLaunchArgument(
        'publisher_num', default_value="1")
    declare_subscriber_num = DeclareLaunchArgument(
        'subscriber_num', default_value="1")
    declare_use_intra_process_comms = DeclareLaunchArgument(
        'use_intra_process_comms', default_value="False")
    declare_use_dedicated_executors = DeclareLaunchArgument(
        'use_dedicated_executors', default_value="True")
    declare_rate = DeclareLaunchArgument(
        'rate',default_value="1")
    declare_point_num = DeclareLaunchArgument(
        'point_num', default_value="1000")
    declare_use_unique_ptr = DeclareLaunchArgument(
        'use_unique_ptr', default_value="False")
    declare_enable_output_address = DeclareLaunchArgument(
        'enable_output_address', default_value="True")
    declare_enable_output_delay = DeclareLaunchArgument(
        'enable_output_delay', default_value="True")

    # composed node
    composed_node = Node(package='ros2_topic_performance',
        executable='composed_npub_nsub',
        parameters=[
            {"publisher_num":publisher_num,
            "subscriber_num":subscriber_num,
            "use_intra_process_comms":use_intra_process_comms,
            "use_dedicated_executors":use_dedicated_executors,
            "rate" : rate,
            "point_num": point_num,
            "use_unique_ptr": use_unique_ptr,
            "enable_output_address":enable_output_address,
            "enable_output_delay":enable_output_delay}, 
        ],
        output="screen")

    ld = LaunchDescription()
    ld.add_action(declare_publisher_num)
    ld.add_action(declare_subscriber_num)
    ld.add_action(declare_use_intra_process_comms)
    ld.add_action(declare_use_dedicated_executors)
    ld.add_action(declare_rate)
    ld.add_action(declare_point_num)
    ld.add_action(declare_use_unique_ptr)
    ld.add_action(declare_enable_output_address)
    ld.add_action(declare_enable_output_delay)
    ld.add_action(composed_node)
    return ld