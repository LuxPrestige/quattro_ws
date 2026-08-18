"""Launch the complete Quattro hardware stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
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


def generate_launch_description() -> LaunchDescription:
    """Build the real-hardware launch description."""
    use_imu = LaunchConfiguration('use_imu')
    use_teleop = LaunchConfiguration('use_teleop')
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

    bringup_share = FindPackageShare('quattro_bringup')
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
                'start_enabled': True,
                'initial_pose_duration': ParameterValue(
                    initial_pose_duration, value_type=float),
                'use_sim_time': False,
            },
        ],
        condition=IfCondition(PythonExpression([
            "'", hardware_control_method,
            "' == 'direct_position'",
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
    start_gait_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller,
            on_exit=[gait_controller],
        )
    )
    stop_on_controller_manager_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager,
            on_exit=[EmitEvent(event=Shutdown(reason='controller_manager exited'))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_file',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'calibration.yaml']),
            description='Machine-specific motor calibration YAML.'),
        DeclareLaunchArgument(
            'hardware_control_method', default_value='direct_position',
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
            'command_controller_name',
            default_value='joint_trajectory_controller',
            description='Controller name present in controller_file.'),
        DeclareLaunchArgument(
            'controller_file',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'hardware_controllers.yaml']),
            description='ros2_control controller configuration.'),
        DeclareLaunchArgument(
            'initial_pose_duration', default_value='5.0',
            description='Seconds used for the initial IK trajectory.'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='Start the BNO085 IMU node.'),
        DeclareLaunchArgument(
            'use_teleop', default_value='true',
            description='Start joystick input and Quattro teleoperation.'),
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
    ])
