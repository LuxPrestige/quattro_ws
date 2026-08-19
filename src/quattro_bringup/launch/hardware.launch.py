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
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml

# Must match the `joints:` order in quattro_bringup/config/hardware_controllers_mit.yaml.
_JOINT_ORDER = [
    'front_left_hip_joint', 'front_left_upper_leg_joint', 'front_left_lower_leg_joint',
    'front_right_hip_joint', 'front_right_upper_leg_joint', 'front_right_lower_leg_joint',
    'back_left_hip_joint', 'back_left_upper_leg_joint', 'back_left_lower_leg_joint',
    'back_right_hip_joint', 'back_right_upper_leg_joint', 'back_right_lower_leg_joint',
]
# Used only if calibration.yaml cannot be read; matches the previous static
# defaults in hardware_controllers_mit.yaml so a missing/unreadable file
# never silently blocks bringup.
_FALLBACK_MIT_KP = 60.0
_FALLBACK_MIT_KD = 0.8


def _mit_gains_from_calibration(calibration_path: str) -> dict:
    """
    Read per-joint MIT kp/kd from calibration.yaml.

    calibration.yaml is the single source of truth for MIT gains: the same
    per-joint kp/kd feed QuattroSystem's activation gain-ramp through the
    URDF (see quattro_description/urdf/quattro.urdf.xacro). Sourcing
    mit_trajectory_controller's steady-state gains from the same file avoids
    a gain discontinuity the moment control hands off from the hardware
    layer to the controller after activation.
    """
    try:
        with open(calibration_path, 'r', encoding='utf-8') as calibration_stream:
            joints = yaml.safe_load(calibration_stream)['joints']
        kp = [float(joints[name]['kp']) for name in _JOINT_ORDER]
        kd = [float(joints[name]['kd']) for name in _JOINT_ORDER]
    except (OSError, KeyError, TypeError, ValueError):
        kp = [_FALLBACK_MIT_KP] * len(_JOINT_ORDER)
        kd = [_FALLBACK_MIT_KD] * len(_JOINT_ORDER)
    return {'kp': kp, 'kd': kd}


def launch_setup(context, *args, **kwargs):
    """Resolve launch configurations and build the real-hardware actions."""
    use_imu = LaunchConfiguration('use_imu')
    use_teleop = LaunchConfiguration('use_teleop')
    start_gait_enabled = LaunchConfiguration('start_gait_enabled')
    staged_initial_pose = LaunchConfiguration('staged_initial_pose')
    initial_pose_duration = LaunchConfiguration('initial_pose_duration')
    calibration_file = LaunchConfiguration('calibration_file')
    controller_file = LaunchConfiguration('controller_file')
    hardware_control_method = LaunchConfiguration('hardware_control_method')
    command_controller_name = LaunchConfiguration('command_controller_name')
    apply_position_gains = LaunchConfiguration('apply_position_gains')
    position_gain = LaunchConfiguration('position_gain')
    velocity_gain = LaunchConfiguration('velocity_gain')
    velocity_integrator_gain = LaunchConfiguration(
        'velocity_integrator_gain')
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
                ' hardware_control_method:=', hardware_control_method,
                ' apply_position_gains:=', apply_position_gains,
                ' position_gain:=', position_gain,
                ' velocity_gain:=', velocity_gain,
                ' velocity_integrator_gain:=', velocity_integrator_gain,
                ' motor_activation_interval_ms:=', motor_activation_interval_ms,
            ]),
            value_type=str,
        )
    }
    # Only takes effect while mit_trajectory_controller is actually loaded by
    # controller_file; harmless otherwise.
    mit_gain_overrides = {
        'mit_trajectory_controller': {
            'ros__parameters': _mit_gains_from_calibration(
                context.perform_substitution(calibration_file)),
        },
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
            mit_gain_overrides,
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
            command_controller_name,
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
                'trajectory_controller_name': command_controller_name,
                'initial_pose_duration': ParameterValue(
                    initial_pose_duration, value_type=float),
                'staged_initial_pose': ParameterValue(
                    staged_initial_pose, value_type=bool),
                'use_sim_time': False,
            },
        ],
        condition=IfCondition(PythonExpression([
            "'", hardware_control_method,
            "' in ('direct_position', 'mit')",
        ])),
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
            'hardware_control_method', default_value='mit',
            description=(
                'GDS68 control method: direct_position, direct_velocity, '
                'direct_torque, or mit. The controller file must use the '
                'matching command interface.')),
        DeclareLaunchArgument(
            'apply_position_gains', default_value='false',
            description=(
                'Write runtime GDS68 position/velocity gains during configure. '
                'False preserves the device values.')),
        DeclareLaunchArgument(
            'position_gain', default_value='0.0',
            description='GDS68 position gain; used only when enabled.'),
        DeclareLaunchArgument(
            'velocity_gain', default_value='0.0',
            description='GDS68 velocity gain; used only when enabled.'),
        DeclareLaunchArgument(
            'velocity_integrator_gain', default_value='0.0',
            description='GDS68 velocity integrator gain; used only when enabled.'),
        DeclareLaunchArgument(
            'motor_activation_interval_ms', default_value='100',
            description=(
                'Delay after each motor reaches closed-loop control before '
                'activating the next motor.')),
        DeclareLaunchArgument(
            'command_controller_name',
            default_value='mit_trajectory_controller',
            description='Controller name present in controller_file.'),
        DeclareLaunchArgument(
            'controller_file',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'hardware_controllers_mit.yaml']),
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
