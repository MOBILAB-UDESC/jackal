from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    node_battery_state_control = Node(
        name='battery_state_control',
        executable='battery_state_control',
        package='clearpath_hardware_interfaces',
        namespace=namespace,
        output='screen',
        arguments=[
            '-s',
            '/etc/clearpath',
        ]
    )

    node_battery_state_estimator = Node(
        name='battery_state_estimator',
        executable='battery_state_estimator',
        package='clearpath_hardware_interfaces',
        namespace=namespace,
        output='screen',
        arguments=[
            '-s',
            '/etc/clearpath',
        ]
    )

    node_wireless_watcher = Node(
        name='wireless_watcher',
        executable='wireless_watcher',
        package='wireless_watcher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'hz': 1.0,
            'dev': '',
            'connected_topic': 'platform/wifi_connected',
            'connection_topic': 'platform/wifi_status',
        }]
    )

    args = [
        DeclareLaunchArgument(
            'namespace',
            default_value='j100_0929',
            description='Top-level namespace.'
        )
    ]

    # Launch nodes and declared arguments
    return LaunchDescription([
        *args,
        node_battery_state_control,
        node_battery_state_estimator,
        node_wireless_watcher,
    ])
