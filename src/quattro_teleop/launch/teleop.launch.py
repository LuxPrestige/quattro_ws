"""Launch Bluetooth game-controller input and Quattro teleoperation."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the joystick teleoperation launch description."""
    config = PathJoinSubstitution(
        [FindPackageShare('quattro_teleop'), 'config', 'switch_pro.yaml'])
    return LaunchDescription([
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='quattro_teleop',
            executable='teleop_node',
            name='quattro_teleop',
            output='screen',
            parameters=[config],
        ),
    ])
