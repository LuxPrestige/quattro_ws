"""Launch the gait generator with a visualization-only joint-state bridge."""

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
    """Build the Quattro gait visualization launch description."""
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    description_share = FindPackageShare('quattro_description')
    xacro_file = PathJoinSubstitution([
        description_share, 'urdf', 'quattro.urdf.xacro'])
    rviz_file = PathJoinSubstitution([
        description_share, 'rviz', 'quattro.rviz'])
    gait_parameters = PathJoinSubstitution([
        FindPackageShare('quattro'), 'config', 'kinematics.yaml'])
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', xacro_file]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true', description='Start RViz2.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='quattro',
            executable='gait_controller',
            output='screen',
            parameters=[gait_parameters, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='quattro_description',
            executable='trajectory_to_joint_state.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
