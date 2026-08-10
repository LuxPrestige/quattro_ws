"""Launch the Quattro BNO085 driver."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the BNO085 launch description."""
    config = PathJoinSubstitution(
        [FindPackageShare('quattro_sensors'), 'config', 'bno085.yaml'])
    return LaunchDescription([
        Node(package='quattro_sensors', executable='bno085_node',
             name='bno085', output='screen', parameters=[config]),
    ])
