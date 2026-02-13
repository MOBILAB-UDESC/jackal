from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg
from nav2_common.launch import ReplaceString


def generate_launch_description():

    camera_remappings = [
        ('~/aligned_depth_to_color/camera_info', 'depth/camera_info'),
        ('~/aligned_depth_to_color/image_raw', 'depth/image'),
        ('~/aligned_depth_to_color/image_raw/compressed', 'depth/image/compressed'),
        ('~/aligned_depth_to_color/image_raw/compressedDepth', 'depth/image/compressedDepth'),
        ('~/aligned_depth_to_color/image_raw/ffmpeg', 'depth/image/ffmpeg'),
        ('~/aligned_depth_to_color/image_raw/theora', 'depth/image/theora'),
        ('~/aligned_depth_to_color/image_raw/zstd', 'depth/image/zstd'),
        ('~/color/camera_info', 'rgb/camera_info'),
        ('~/color/image_raw', 'rgb/image'),
        ('~/color/image_raw/compressed', 'rgb/image/compressed'),
        ('~/color/image_raw/compressedDepth', 'rgb/image/compressedDepth'),
        ('~/color/image_raw/ffmpeg', 'rgb/image/ffmpeg'),
        ('~/color/image_raw/theora', 'rgb/image/theora'),
        ('~/color/image_raw/zstd', 'rgb/image/zstd'),
        ('~/color/metadata', 'rgb/metadata'),
        ('~/depth/camera_info', 'depth_raw/camera_info'),
        ('~/depth/image_rect_raw', 'depth_raw/image'),
        ('~/depth/image_rect_raw/compressed', 'depth_raw/image/compressed'),
        ('~/depth/image_rect_raw/compressedDepth', 'depth_raw/image/compressedDepth'),
        ('~/depth/image_rect_raw/ffmpeg', 'depth_raw/image/ffmpeg'),
        ('~/depth/image_rect_raw/theora', 'depth_raw/image/theora'),
        ('~/depth/image_rect_raw/zstd', 'depth_raw/image/zstd'),
        ('~/depth/metadata', 'depth_raw/metadata'),
        ('~/extrinsics/depth_to_color', 'extrinsics/depth_to_color'),
    ]

    bringup_pkg_path = get_package_share_directory('jackal_bringup')

    prefix = LaunchConfiguration('prefix')
    sensor_params = LaunchConfiguration('sensor_params')

    nodes = []

    sensor_params = ReplaceString(
        source_file=sensor_params,
        replacements={
            '<robot_prefix>': prefix,
        },
    )

    ouster_driver_node = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name='os_driver',
        namespace='ouster',
        parameters=[sensor_params],
        output='screen',
        remappings=[
            ('scan', '/scan'),
        ],
    )

    ouster_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ouster_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    ouster_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ouster_driver_node, goal_state='inactive',
            entities=[
                LogInfo(msg="os_driver activating..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(ouster_driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        )
    )

    # Lidar 3D Ouster
    nodes += [
        ouster_driver_node,
        ouster_configure_event,
        ouster_activate_event
    ]

    camera_0_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera_0_node',
        namespace='camera_0',
        parameters=[sensor_params],
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level',
            'info',
        ],
        emulate_tty=True,
        remappings=camera_remappings
    )

    nodes += [
        camera_0_node
    ]

    args = [
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix to prepend to all Jackal link and joint names'
        ),
        DeclareLaunchArgument(
            'sensor_params',
            default_value=PathJoinSubstitution([bringup_pkg_path, 'config', 'sensor_params.yaml']),
            description='Top-level namespace'
        )
    ]

    return LaunchDescription([
        *args,
        *nodes
    ])
