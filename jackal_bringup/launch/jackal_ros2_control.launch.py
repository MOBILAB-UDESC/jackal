from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
import os


def replace_prefix(source, prefix_cfg):
    file = ReplaceString(
        source_file=source,
        replacements={'<robot_prefix>': prefix_cfg},
    )
    return file


def launch_setup(context, *args, **kwargs):
    """
        Set up ROS 2 control for Jackal and the selected robotic arm and gripper.

        Controllers are launched in sequence to ensure proper initialization:
            1. controller_manager: if use_sim_time is false
            2. joint_state_broadcaster
            3. diff_drive_controller: after joint_state_broadcaster exists
            4. arm_controller: after diff_drive_controller exists if 'arm' is enabled
            5. gripper_controller: after arm_controller exits if 'gripper' is enabled
    """

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    nodes = []

    # Configure topic remappings for real hardware communication
    remappings = []
    if use_sim_time.perform(context) == 'false':
        remap_cmd_drive = namespace.perform(context) + '/platform/motors/cmd_drive'
        remap_feedback = namespace.perform(context) + '/platform/motors/feedback'
        remappings = [
            ('platform/motors/cmd_drive', remap_cmd_drive),
            ('platform/motors/feedback', remap_feedback)
        ]

    ros2_control_params = replace_prefix(
        LaunchConfiguration('ros2_control_params'),
        LaunchConfiguration('prefix')
    )

    controller_manager_node = Node(
        condition=UnlessCondition(use_sim_time),
        package='controller_manager',
        executable='ros2_control_node',
        output="both",
        parameters=[ros2_control_params, {'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster',
        output='screen',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    jackal_drive_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='jackal_drive_controller',
        output='screen',
        arguments=[f'jackal_drive_base_controller', '--param-file', ros2_control_params],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    # Jackal ROS 2 control nodes
    nodes += [
        controller_manager_node,
        joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=jackal_drive_base_controller,
            )
        ),
    ]

    # Check if an arm is selected
    arm_name = LaunchConfiguration('arm').perform(context)
    gripper_name = LaunchConfiguration('gripper').perform(context)

    if not arm_name:
        return nodes

    # Check for arm-specific setup launch file
    arm_pkg = get_package_share_directory(f'{arm_name}_description')
    arm_setup_launch = PathJoinSubstitution(
        [arm_pkg, 'launch', f'{arm_name}_setup.launch.py']
    ).perform(context)

    if os.path.exists(arm_setup_launch):
        arm_setup_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_setup_launch),
            condition=UnlessCondition(use_sim_time),
        )
        nodes += [arm_setup_node]

    # arms_bringup pkg contains config files for all supported arms + grippers
    arm_bringup_pkg_path = get_package_share_directory('arms_bringup')
    arm_ros2_control_params = replace_prefix(
        PathJoinSubstitution([arm_bringup_pkg_path, 'config', 'ros2_control.yaml']),
        LaunchConfiguration('arm_prefix')
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name=f'{arm_name}_trajectory_controller',
        output='screen',
        arguments=[f'{arm_name}_arm_controller', '--param-file', arm_ros2_control_params],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    nodes.append(arm_controller)

    if gripper_name:
        gripper_controller = Node(
            package='controller_manager',
            executable='spawner',
            name=f'{gripper_name}_gripper_controller',
            output='screen',
            arguments=[
                f'{gripper_name}_gripper_controller',
                '--param-file',
                arm_ros2_control_params
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

        nodes.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=arm_controller,
                    on_exit=[gripper_controller],
                )
            )
        )

    return nodes


def generate_launch_description():
    bringup_pkf_path = get_package_share_directory('jackal_bringup')

    args = [
        DeclareLaunchArgument(
            'arm',
            default_value='',
            choices=[
                '',  # No arm
                'gen3_lite',
                'unitree_d1',
                'unitree_z1',
            ],
            description='Arm model to attach to Jackal'
        ),
        DeclareLaunchArgument(
            'arm_prefix',
            default_value='arm_',
            description='Prefix to prepend to all arm link and joint names'
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='',
            choices=[
                '',  # No gripper
                'kinova_2f_lite',
                'robotiq_2f_85',
            ],
            description='Gripper model to attach to the arm'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='j100_0929',
            description='Top-level namespace'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix to prepend to all Jackal link and joint names'
        ),
        DeclareLaunchArgument(
            'ros2_control_params',
            default_value=PathJoinSubstitution([bringup_pkf_path, 'config', 'ros2_control.yaml']),
            description='Absolute path to the ROS 2 control configuration YAML file'
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
        OpaqueFunction(
            function=launch_setup,
        )
    ])
