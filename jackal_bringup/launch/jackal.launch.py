from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import (
    IfCondition,
    UnlessCondition
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """
        Set up and return a list of ROS 2 launch actions for bringing up Jackal.

        This function performs the following tasks:
        - Robot State Publisher
        - Gazebo simulation: if use_sim_time is True
        - ROS 2 control
        - EKF node
        - Joystick Teleoperation
        - Twist Mux for multiple velocity command sources
        - Rviz2

        Returns:
            list: List of nodes ready for execution.
    """
    arm = LaunchConfiguration('arm').perform(context)
    arm_prefix = LaunchConfiguration('arm_prefix').perform(context)
    gripper = LaunchConfiguration('gripper')
    prefix = LaunchConfiguration('prefix').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_name = 'jackal'

    description_pkg_path = get_package_share_directory('jackal_description')
    bringup_pkf_path = get_package_share_directory('jackal_bringup')

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([description_pkg_path, 'urdf', 'jackal.urdf.xacro']),
        ' arm:=', arm,
        ' arm_prefix:=', arm_prefix,
        ' gripper:=', gripper,
        ' name:=', robot_name,
        ' prefix:=', prefix,
        ' sim_gazebo:=', use_sim_time,
        ' use_camera:=', LaunchConfiguration('use_camera'),
        ' use_lidar:=', LaunchConfiguration('use_lidar'),
    ])

    nodes = []

    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
    )

    # Joint state GUI for testing
    joint_state_publisher_gui = Node(
        condition=UnlessCondition(use_sim_time),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # Add robot description and TF broadcasting
    nodes += [
        robot_state_node,
        # joint_state_publisher_gui
    ]

    gazebo_spawn_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [bringup_pkf_path, 'launch', 'jackal_gazebo.launch.py']
        )),
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
        launch_arguments={
            'prefix': prefix,
            'robot_name': robot_name,
            'use_sim_time': use_sim_time,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'Y': LaunchConfiguration('Y')
        }.items()
    )

    # Gazebo simulation
    nodes += [
        gazebo_spawn_node
    ]

    robot_controllers = PathJoinSubstitution([bringup_pkf_path, 'config', 'ros2_control.yaml'])

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [bringup_pkf_path, 'launch', 'jackal_ros2_control.launch.py'])),
        launch_arguments={
            'arm_prefix': arm_prefix,
            'prefix': prefix,
            'use_sim_time': use_sim_time,
            'ros2_control_params': robot_controllers,
            'prefix': prefix,
        }.items()
    )

    ekf_file = PathJoinSubstitution([bringup_pkf_path, 'config', 'ekf.yaml'])

    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [bringup_pkf_path, 'launch', 'jackal_ekf.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ekf_params': ekf_file,
            'prefix': prefix
        }.items()
    )

    # Load ROS 2 control and odometry
    nodes += [
        ros2_control_launch,
        ekf_node
    ]

    twist_joy_config = PathJoinSubstitution([bringup_pkf_path, 'config', 'twist_joy.yaml'])

    node_joy = Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen',
        name='joy_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            twist_joy_config
        ],
        respawn=True,
    )

    node_teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            twist_joy_config
        ],
        remappings=[
            ('cmd_vel', 'joy_teleop/cmd_vel'),
        ]
    )

    twist_mux_config = PathJoinSubstitution([bringup_pkf_path, 'config', 'twist_mux.yaml'])

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'use_stamped': True},
            twist_mux_config
        ],
        remappings=[
            ('cmd_vel_out', 'diff_drive_base_controller/cmd_vel'),
        ]
    )

    # Joystick teleoperation
    nodes += [
        node_joy,
        node_teleop_twist_joy,
        node_twist_mux
    ]

    rviz2_path = PathJoinSubstitution([bringup_pkf_path, 'rviz', 'jackal.rviz'])

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz2_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Visualization
    nodes += [
        rviz2_node
    ]

    return nodes


def generate_launch_description():
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
            default_value=PathJoinSubstitution(
                [get_package_share_directory('jackal_bringup'), 'config', 'ros2_control.yaml']),
            description='Absolute path to the ROS 2 control configuration YAML file'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use a camera'
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use a lidar'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use simulation time'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Robot initial pose x'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Robot initial pose y'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.06',
            description='Robot initial pose z'
        ),
        DeclareLaunchArgument(
            'Y',
            default_value='0.0',
            description='Robot initial yaw (rotation around Z axis)'
        ),
    ]

    # Launch nodes and declared arguments
    return LaunchDescription([
        *args,
        OpaqueFunction(function=launch_setup),
    ])
