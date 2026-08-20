"""Launch Quattro in Gazebo Harmonic with ros2_control and RViz2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    """Resolve launch configurations and build the simulation actions."""
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    hardware_control_method = LaunchConfiguration('hardware_control_method')
    use_gain_scheduler = LaunchConfiguration('use_gain_scheduler')

    gazebo_share = get_package_share_directory('quattro_gazebo')
    description_share = get_package_share_directory('quattro_description')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(gazebo_share, 'worlds', 'flat.world.sdf')
    method = context.perform_substitution(hardware_control_method)
    gain_scheduler_requested = (
        context.perform_substitution(use_gain_scheduler) == 'true')
    # mit uses quattro_controllers/MitTrajectoryController in its Gazebo
    # "effort_emulation" mode (docs/packages/quattro_controllers.md) since
    # gz_ros2_control's stock GazeboSimSystem has no native kp/kd interface.
    # Every other value keeps the original position-only simulation path
    # unchanged (quattro_description/urdf/quattro.urdf.xacro).
    is_mit = method == 'mit'
    controller_file = os.path.join(
        gazebo_share, 'config',
        'gazebo_controllers_mit.yaml' if is_mit else 'gazebo_controllers.yaml')
    command_controller_name = (
        'mit_trajectory_controller' if is_mit else 'joint_trajectory_controller')
    gain_scheduler_parameters = os.path.join(
        gazebo_share, 'config', 'gain_scheduler.yaml')
    xacro_file = os.path.join(
        description_share, 'urdf', 'quattro.urdf.xacro')
    rviz_file = os.path.join(description_share, 'rviz', 'quattro.rviz')
    gait_parameters = os.path.join(
        get_package_share_directory('quattro'), 'config', 'kinematics.yaml')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', xacro_file,
                ' simulation:=true',
                ' hardware_control_method:=', hardware_control_method,
                ' simulation_controllers:=', controller_file,
            ]),
            value_type=str,
        )
    }

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -v 3 {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v 3 {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(headless),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    spawn_robot = TimerAction(
        period=2.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'quattro',
                '-topic', 'robot_description',
                '-allow_renaming', 'false',
                '-z', '0.325',
            ],
            output='screen',
        )],
    )
    controller_spawners = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '30',
                ],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    command_controller_name,
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '30',
                ],
                output='screen',
            ),
        ],
    )
    gain_scheduler_actions = []
    if is_mit and gain_scheduler_requested:
        gain_scheduler_actions.append(TimerAction(
            period=5.0,
            actions=[Node(
                package='quattro_core_ros',
                executable='gain_scheduler_node',
                output='screen',
                parameters=[gain_scheduler_parameters, {'use_sim_time': True}],
            )],
        ))
    gait_controller = TimerAction(
        period=6.0,
        actions=[Node(
            package='quattro',
            executable='gait_controller',
            output='screen',
            parameters=[
                gait_parameters,
                {
                    'trajectory_controller_name': command_controller_name,
                    'use_sim_time': True,
                },
            ],
        )],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return [
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.dirname(description_share)),
        gazebo_gui,
        gazebo_headless,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        controller_spawners,
        *gain_scheduler_actions,
        gait_controller,
        rviz,
    ]


def generate_launch_description() -> LaunchDescription:
    """Build the Gazebo simulation launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true', description='Start RViz2.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run only the Gazebo server without its GUI.'),
        DeclareLaunchArgument(
            'hardware_control_method', default_value='direct_position',
            description=(
                'Simulation command interface: direct_position (default, '
                'standard joint_trajectory_controller/position, unchanged '
                'from before) or mit (quattro_controllers/'
                "MitTrajectoryController in Gazebo 'effort_emulation' mode, "
                'docs/packages/quattro_controllers.md). Other real-hardware '
                'values (direct_velocity/direct_torque) are not supported '
                'in simulation.')),
        DeclareLaunchArgument(
            'use_gain_scheduler', default_value='true',
            description=(
                'Start quattro_core_ros/gain_scheduler_node when '
                'hardware_control_method=mit, switching MIT joint gain '
                'between swing/stance profiles (docs/control/'
                'gain_tuning.md). Ignored otherwise.')),
        OpaqueFunction(function=launch_setup),
    ])
