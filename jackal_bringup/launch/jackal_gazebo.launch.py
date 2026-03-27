from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString, RewrittenYaml


def launch_setup(context):

    arm = LaunchConfiguration('arm', default='').perform(context)
    arm_prefix = LaunchConfiguration('arm_prefix', default='arm_').perform(context)
    robot_prefix = LaunchConfiguration('prefix', default='').perform(context)

    config_file = PathJoinSubstitution([
        get_package_share_directory('jackal_bringup'),
        'config',
        'bridge.yaml'
    ])
    config_file = ReplaceString(
        source_file=config_file,
        replacements={'<robot_prefix>': robot_prefix},
    )

    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            ['config_file:=', config_file],
        ],
    )

    nodes_to_start = [gazebo_bridge_node]

    if arm:
        camera_topic = f'/{arm_prefix}camera_1'

        arms_sensors_config = [
            f'{camera_topic}/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{camera_topic}/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{camera_topic}/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            f'{camera_topic}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ]

        arm_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='arm_camera_bridge',
            output='screen',
            arguments=arms_sensors_config,
            remappings=[
                (f'{camera_topic}/image', f'{camera_topic}/rgb/image'),
                (f'{camera_topic}/depth_image', f'{camera_topic}/depth/image'),
                (f'{camera_topic}/camera_info', f'{camera_topic}/rgb/camera_info'),
            ]
        )

        nodes_to_start.append(arm_bridge_node)

    return nodes_to_start


def generate_launch_description():
    """Spawn the robot and optionally bridging its sensors."""

    gazebo_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-allow_renaming', 'true',
            '-name', LaunchConfiguration('robot_name', default='jackal'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x', default='0.0'),
            '-y', LaunchConfiguration('y', default='0.0'),
            '-z', LaunchConfiguration('z', default='0.06'),
            '-Y', LaunchConfiguration('Y', default='0.0'),
        ],
    )

    return LaunchDescription([
        gazebo_spawn_node,
        OpaqueFunction(function=launch_setup)
    ])
