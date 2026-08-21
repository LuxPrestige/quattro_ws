"""Launch the complete Quattro hardware stack."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Resolve launch configurations and build the real-hardware actions."""
    use_imu = LaunchConfiguration('use_imu')
    use_teleop = LaunchConfiguration('use_teleop')
    start_gait_enabled = LaunchConfiguration('start_gait_enabled')
    staged_initial_pose = LaunchConfiguration('staged_initial_pose')
    initial_pose_duration = LaunchConfiguration('initial_pose_duration')
    calibration_file = LaunchConfiguration('calibration_file')
    controller_file = LaunchConfiguration('controller_file')
    motor_activation_interval_ms = LaunchConfiguration(
        'motor_activation_interval_ms')

    description_share = FindPackageShare('quattro_description')
    xacro_file = PathJoinSubstitution([
        description_share, 'urdf', 'quattro.urdf.xacro'])
    gait_parameters = PathJoinSubstitution([
        FindPackageShare('quattro'), 'config', 'kinematics.yaml'])
    imu_parameters = PathJoinSubstitution([
        FindPackageShare('quattro_sensors'), 'config', 'bno085.yaml'])
    teleop_parameters = PathJoinSubstitution([
        FindPackageShare('quattro_teleop'), 'config', 'switch_pro.yaml'])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', xacro_file,
                ' simulation:=false',
                ' calibration_file:=', calibration_file,
                ' motor_activation_interval_ms:=', motor_activation_interval_ms,
            ]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
    )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description,
            controller_file,
            {'update_rate': 100, 'use_sim_time': False},
        ],
    )
    hardware_spawner = Node(
        package='controller_manager',
        executable='hardware_spawner',
        arguments=[
            'QuattroSystem',
            '--activate',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--switch-timeout', '30',
        ],
        output='screen',
    )
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--switch-timeout', '30',
        ],
        output='screen',
    )
    gait_controller = Node(
        package='quattro',
        executable='gait_controller',
        output='screen',
        parameters=[
            gait_parameters,
            {
                'control_frequency': 100.0,
                'start_enabled': ParameterValue(
                    start_gait_enabled, value_type=bool),
                'trajectory_controller_name': 'joint_trajectory_controller',
                'initial_pose_duration': ParameterValue(
                    initial_pose_duration, value_type=float),
                'staged_initial_pose': ParameterValue(
                    staged_initial_pose, value_type=bool),
                'use_sim_time': False,
            },
        ],
    )
    imu = Node(
        package='quattro_sensors',
        executable='bno085_node',
        name='bno085',
        output='screen',
        parameters=[imu_parameters, {'use_sim_time': False}],
        condition=IfCondition(use_imu),
    )
    game_controller = Node(
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
        output='screen',
        parameters=[teleop_parameters, {'use_sim_time': False}],
        condition=IfCondition(use_teleop),
    )
    teleop = Node(
        package='quattro_teleop',
        executable='teleop_node',
        name='quattro_teleop',
        output='screen',
        parameters=[
            teleop_parameters,
            {'start_stepping': True, 'use_sim_time': False},
        ],
        condition=IfCondition(use_teleop),
    )
    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=hardware_spawner,
            on_exit=[joint_state_broadcaster],
        )
    )

    start_trajectory_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[joint_trajectory_controller],
        )
    )

    def start_gait_or_shutdown(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason='command controller failed to start'))]
        return [gait_controller]

    start_gait_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller,
            on_exit=start_gait_or_shutdown,
        )
    )
    stop_on_controller_manager_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager,
            on_exit=[EmitEvent(event=Shutdown(reason='controller_manager exited'))],
        )
    )

    return [
        robot_state_publisher,
        controller_manager,
        hardware_spawner,
        imu,
        game_controller,
        teleop,
        start_joint_state_broadcaster,
        start_trajectory_controller,
        start_gait_controller,
        stop_on_controller_manager_exit,
    ]


def generate_launch_description() -> LaunchDescription:
    """Build the real-hardware launch description."""
    bringup_share = FindPackageShare('quattro_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_file',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'calibration.yaml']),
            description='Machine-specific motor calibration YAML.'),
        DeclareLaunchArgument(
            'motor_activation_interval_ms', default_value='100',
            description=(
                'Delay after each motor reaches closed-loop control before '
                'activating the next motor.')),
        DeclareLaunchArgument(
            'controller_file',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'hardware_controllers.yaml']),
            description='ros2_control controller configuration.'),
        DeclareLaunchArgument(
            'initial_pose_duration', default_value='5.0',
            description='Seconds used for the initial IK trajectory.'),
        DeclareLaunchArgument(
            'start_gait_enabled', default_value='false',
            description=(
                'Immediately command the initial gait pose after controller '
                'activation. Keep false for hardware bringup and gain tests.')),
        DeclareLaunchArgument(
            'staged_initial_pose', default_value='true',
            description=(
                'Move one three-joint leg at a time during the initial pose '
                'transition. Recommended for real hardware.')),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='Start the BNO085 IMU node.'),
        DeclareLaunchArgument(
            'use_teleop', default_value='true',
            description='Start joystick input and Quattro teleoperation.'),
        OpaqueFunction(function=launch_setup),
    ])
