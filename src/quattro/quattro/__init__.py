"""High-level control algorithms for the Quattro robot."""

from quattro.gait import GaitGenerator, GaitParameters
from quattro.kinematics import QuadrupedKinematics, RobotGeometry

__all__ = [
    'GaitGenerator',
    'GaitParameters',
    'QuadrupedKinematics',
    'RobotGeometry',
]
