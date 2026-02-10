from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    slam_launch_path = PathJoinSubstitution(
        [get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    prefix = LaunchConfiguration('prefix')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # --- Handle placeholders inside slam_params_file ---
    # Placeholders (<robot_prefix>) are replaced so can match the robot link names.
    slam_params_file = ReplaceString(
        source_file=slam_params_file,
        replacements={
            '<robot_prefix>': prefix,
        },
    )

    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    rviz2_path = PathJoinSubstitution([
        get_package_share_directory('jackal_navigation'),
        'rviz',
        'jackal_slam.rviz'
    ])

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz2_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    args = [
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix added to the jackal link and joint names'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [get_package_share_directory('jackal_navigation'), 'config', 'slam.yaml']
            ),
            description='Full path to the slam_toolbox config file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use simulation time'
        ),
    ]

    return LaunchDescription([
        *args,
        start_slam_toolbox_cmd,
        rviz2_node,
    ])


# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     EmitEvent,
#     LogInfo,
#     RegisterEventHandler
# )
# from launch.conditions import IfCondition
# from launch.events import matches_action
# from launch.substitutions import (
#     AndSubstitution,
#     LaunchConfiguration,
#     NotSubstitution,
#     PathJoinSubstitution
# )
# from launch_ros.actions import LifecycleNode, Node
# from launch_ros.events.lifecycle import ChangeState
# from lifecycle_msgs.msg import Transition
# from launch_ros.event_handlers import OnStateTransition
# from nav2_common.launch import ReplaceString


# def generate_launch_description():
#     prefix = LaunchConfiguration('prefix')
#     slam_params = LaunchConfiguration('slam_params')
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     # --- Handle placeholders inside slam_params ---
#     # Placeholders (<robot_prefix>) are replaced so can match the robot link names.
#     slam_params = ReplaceString(
#         source_file=slam_params,
#         replacements={
#             '<robot_prefix>': prefix,
#         },
#     )

#     start_async_slam_toolbox_node = LifecycleNode(
#         package='slam_toolbox',
#         executable='async_slam_toolbox_node',
#         name='slam_toolbox',
#         output='screen',
#         parameters=[
#             slam_params,
#             {
#                 'use_lifecycle_manager': False,
#                 'use_sim_time': use_sim_time
#             }
#         ],
#         namespace=''
#     )

#     configure_event = EmitEvent(
#         event=ChangeState(
#             lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
#             transition_id=Transition.TRANSITION_CONFIGURE
#         ),
#     )

#     activate_event = RegisterEventHandler(
#         OnStateTransition(
#             target_lifecycle_node=start_async_slam_toolbox_node,
#             start_state="configuring",
#             goal_state="inactive",
#             entities=[
#                 LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
#                 EmitEvent(event=ChangeState(
#                     lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
#                     transition_id=Transition.TRANSITION_ACTIVATE
#                 ))
#             ]
#         ),
#     )

#     rviz2_path = PathJoinSubstitution([
#         get_package_share_directory('jackal_navigation'),
#         'rviz',
#         'jackal_slam.rviz'
#     ])

#     rviz2_node = Node(
#         condition=IfCondition(LaunchConfiguration('rviz')),
#         package='rviz2',
#         executable='rviz2',
#         output='screen',
#         arguments=['-d', rviz2_path],
#         parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
#     )

#     args = [
#         DeclareLaunchArgument(
#             'prefix',
#             default_value='',
#             description='Prefix added to the jackal link and joint names'
#         ),
#         DeclareLaunchArgument(
#             'rviz',
#             default_value='false',
#             choices=['true', 'false'],
#             description='Whether to execute rviz2'
#         ),
#         DeclareLaunchArgument(
#             'slam_params',
#             default_value=PathJoinSubstitution(
#                 [get_package_share_directory('jackal_navigation'), 'config', 'slam.yaml']
#             ),
#             description='Full path to the slam_toolbox config file'
#         ),
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='true',
#             choices=['true', 'false'],
#             description='Whether to use simulation time'
#         ),
#     ]

#     return LaunchDescription([
#         *args,
#         start_async_slam_toolbox_node,
#         configure_event,
#         activate_event,
#         rviz2_node,
#     ])
