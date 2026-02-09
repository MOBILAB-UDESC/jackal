from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess
)
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration
)
from launch_ros.actions import Node


def generate_launch_description():
    node_micro_ros_agent = Node(
        name='micro_ros_agent',
        executable='micro_ros_agent',
        package='micro_ros_agent',
        output='screen',
        arguments=[
            'serial',
            '--dev',
            '/dev/clearpath/j100',
        ]
    )

    namespace = LaunchConfiguration('namespace')
    domain_id = LaunchConfiguration('domain_id')

    process_configure_mcu = ExecuteProcess(
        shell=True,
        cmd=[
            # Temporarily set domain ID to 0 to communicate with freshly booted MCU
            ['export ROS_DOMAIN_ID=0;'],
            [
                FindExecutable(name='ros2'),
                ' service call platform/mcu/configure',
                ' clearpath_platform_msgs/srv/ConfigureMcu',
                ' "{domain_id: ',
                domain_id,
                ',',
                ' robot_namespace: \'',
                namespace,
                '\'}"'
            ],
        ]
    )

    imu_remap_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_imu_node',
        output='screen',
        arguments=[
            [namespace, '/sensors/imu_0/data_raw'], 'imu_0/data_raw'
        ]

    )

    mag_remap_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_mag_node',
        output='screen',
        arguments=[
            [namespace, '/sensors/imu_0/magnetic_field'], 'imu_0/mag'
        ]

    )

    gps_remap_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_gps_node',
        output='screen',
        arguments=[
            [namespace, '/sensors/gps_0/nmea_sentence'], 'gps_0/nmea_sentence'
        ]

    )

    args = [
        DeclareLaunchArgument(
            'domain_id',
            default_value='7',
            description='DDS domain id'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='j100_0929',
            description='Top-level namespace'
        )
    ]

    return LaunchDescription([
        *args,
        node_micro_ros_agent,
        process_configure_mcu,
        imu_remap_node,
        mag_remap_node,
        gps_remap_node
    ])
