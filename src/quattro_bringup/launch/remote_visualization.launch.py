"""Visualize actual and commanded Quattro joint positions on a remote desktop."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the desktop-only hardware visualization launch description."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    description_share = FindPackageShare('quattro_description')
    xacro_file = PathJoinSubstitution([
        description_share, 'urdf', 'quattro.urdf.xacro'])
    rviz_file = PathJoinSubstitution([
        description_share, 'rviz', 'hardware_remote.rviz'])
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', xacro_file]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock.'),
        Node(
            package='quattro_description',
            executable='trajectory_to_joint_state.py',
            name='target_joint_state_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('joint_states', 'target_joint_states')],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='target_robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                {'frame_prefix': 'target/', 'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('joint_states', 'target_joint_states'),
                ('robot_description', 'target_robot_description'),
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='target_base_transform',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'target/base_link',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='hardware_remote_rviz',
            arguments=['-d', rviz_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
