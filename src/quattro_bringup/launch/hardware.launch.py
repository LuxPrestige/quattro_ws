"""
Launch the Quattro hardware stack.

This file only starts processes. Startup *ordering* lives in
``quattro_bringup.bringup_manager``, and the GIM6010 startup sequence itself
lives in ``QuattroSystem`` -- deliberately not in a chain of spawner exits
here, which cannot tell a spawner that succeeded from one that gave up.

The one process that does start off another's exit is gait_controller, and
it is gated on ``bringup_manager``'s exit *code*: 0 only when every stage,
including joint_trajectory_controller activation, actually succeeded.

Reaching the end of this launch file does not mean the robot is ready; the
bringup manager's own READY log line does. Gait is off unless use_gait is
set: bringup leaves the robot holding position.
"""

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
    use_gait = LaunchConfiguration('use_gait')
    start_gait_enabled = LaunchConfiguration('start_gait_enabled')
    staged_initial_pose = LaunchConfiguration('staged_initial_pose')
    initial_pose_duration = LaunchConfiguration('initial_pose_duration')
    calibration_file = LaunchConfiguration('calibration_file')
    controller_file = LaunchConfiguration('controller_file')

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
    # Owns the whole startup state machine: configure -> activate ->
    # verify -> joint_state_broadcaster -> verify /joint_states ->
    # joint_trajectory_controller -> READY.
    bringup_manager = Node(
        package='quattro_bringup',
        executable='bringup_manager',
        name='bringup_manager',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    # Started only when explicitly asked for (use_gait), and only after
    # bringup reports READY -- see start_gait_when_ready below. READY is a
    # holding robot, not a walking one.
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
        condition=IfCondition(use_gait),
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

    stop_on_controller_manager_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager,
            on_exit=[EmitEvent(event=Shutdown(reason='controller_manager exited'))],
        )
    )

    def on_bringup_finished(event, _context):
        """Start gait only if bringup actually reached READY."""
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='bringup failed'))]
        # gait_controller carries IfCondition(use_gait), so this is a no-op
        # unless gait was explicitly asked for.
        return [gait_controller]

    # This is a dependency, not a startup state machine: gait_controller
    # publishes its staged initial-pose trajectory as soon as it has
    # /joint_states, and joint_trajectory_controller drops trajectories
    # silently while inactive (subscriber_is_active_). Starting the two in
    # parallel therefore loses the initial pose entirely and leaves the
    # robot to jump to the gait stance later, so gait must not start until
    # bringup has confirmed JTC is active.
    start_gait_when_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=bringup_manager,
            on_exit=on_bringup_finished,
        )
    )

    return [
        robot_state_publisher,
        controller_manager,
        bringup_manager,
        imu,
        game_controller,
        teleop,
        stop_on_controller_manager_exit,
        start_gait_when_ready,
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
            'controller_file',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'hardware_controllers.yaml']),
            description='ros2_control controller configuration.'),
        DeclareLaunchArgument(
            'use_gait', default_value='false',
            description=(
                'Start the gait controller process. Hardware bringup does '
                'not need it: READY means the robot holds position.')),
        DeclareLaunchArgument(
            'initial_pose_duration', default_value='5.0',
            description='Seconds used for the initial IK trajectory.'),
        DeclareLaunchArgument(
            'start_gait_enabled', default_value='false',
            description=(
                'Immediately command the initial gait pose once the gait '
                'controller starts. Requires use_gait:=true. Keep false for '
                'hardware bringup and gain tests.')),
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
