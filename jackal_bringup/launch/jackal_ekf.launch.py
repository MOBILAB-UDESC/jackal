from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    """
        Prepare the EKF YAML configuration file by replacing the <robot_prefix> placeholder
        with the actual prefix for a robot and creates a launch node for the EKF.
    """
    bringup_pkf_path = get_package_share_directory('jackal_bringup')

    ekf_params = LaunchConfiguration('ekf_params')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_params = ReplaceString(
        source_file=ekf_params,
        replacements={'<robot_prefix>': LaunchConfiguration('prefix')},
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, ekf_params],
    )

    args = [
        DeclareLaunchArgument(
            'ekf_params',
            default_value=PathJoinSubstitution([bringup_pkf_path, 'config', 'ekf.yaml']),
            description='Absolute path to the EKF configuration YAML file'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix to prepend to all Jackal link and joint names'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use simulation time'
        )
    ]

    return LaunchDescription([
        *args,
        ekf_node
    ])
