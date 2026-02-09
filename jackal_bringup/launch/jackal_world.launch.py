import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo world and bridge clock topic."""
    bringup_pkf_path = get_package_share_directory('jackal_bringup')

    ament_prefix_path = os.getenv('AMENT_PREFIX_PATH', '')
    packages_paths = [os.path.join(p, 'share') for p in ament_prefix_path.split(':')]

    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            ':'.join(packages_paths),
            ':',
            os.getenv('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    gazebo_world_path = PathJoinSubstitution(
        [bringup_pkf_path, 'worlds', [LaunchConfiguration('world_name'), '.sdf']])

    gazebo_world_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments={
            'render_engine': 'ogre2',
            'gz_args': ['-r -v 1 ', gazebo_world_path],
            'on_exit_shutdown': 'true'
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock']
    )

    args = [
        DeclareLaunchArgument(
            'world_name',
            default_value='empty',
            choices=[
                'empty',
                'mobilab',
                'playground'
            ],
            description='Gazebo world file (custom worlds should be added to the "worlds" folder of this package'
        ),
    ]

    return LaunchDescription([
        *args,
        gz_sim_resource_path,
        gazebo_world_node,
        clock_bridge,
    ])
