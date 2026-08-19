"""ROS node converting velocity commands into joint trajectories."""

import math

from geometry_msgs.msg import PoseStamped, Twist
from quattro.gait import GaitGenerator, GaitParameters
from quattro.kinematics import (
    QuadrupedKinematics,
    RobotGeometry,
    UnreachableTargetError,
)
from quattro.pose_controller import quaternion_to_rpy
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def ramp_value(
        current: float, target: float, rate: float, dt: float) -> float:
    """Move one command value toward its target at a bounded rate."""
    difference = target - current
    step = rate * dt
    if abs(difference) <= step:
        return target
    return current + math.copysign(step, difference)


def staged_joint_targets(
        current: list[float], target: list[float],
        joints_per_stage: int = 3) -> list[list[float]]:
    """Build targets that move one contiguous joint group per stage."""
    if (len(current) != len(target) or not current or
            joints_per_stage <= 0 or len(current) % joints_per_stage != 0):
        raise ValueError('invalid staged joint target dimensions')
    staged = list(current)
    targets = []
    for start in range(0, len(staged), joints_per_stage):
        staged[start:start + joints_per_stage] = target[
            start:start + joints_per_stage]
        targets.append(list(staged))
    return targets


class GaitController(Node):
    """Run a safe, timeout-aware trot trajectory generator."""

    def __init__(self) -> None:
        super().__init__('gait_controller')
        geometry_defaults = RobotGeometry()
        geometry_values = {
            name: float(self.declare_parameter(
                name, getattr(geometry_defaults, name)).value)
            for name in geometry_defaults.__dataclass_fields__
        }
        gait_defaults = GaitParameters()
        gait_values = {
            name: float(self.declare_parameter(
                name, getattr(gait_defaults, name)).value)
            for name in gait_defaults.__dataclass_fields__
        }
        self._control_frequency = float(self.declare_parameter(
            'control_frequency', 100.0).value)
        self._command_timeout = float(self.declare_parameter(
            'command_timeout', 0.5).value)
        self._velocity_ramp_rate = float(self.declare_parameter(
            'velocity_ramp_rate', 0.5).value)
        self._stop_ramp_rate = float(self.declare_parameter(
            'stop_ramp_rate', 0.3).value)
        self._initial_pose_duration = float(self.declare_parameter(
            'initial_pose_duration', 5.0).value)
        self._staged_initial_pose = bool(self.declare_parameter(
            'staged_initial_pose', False).value)
        self._pid_kp = float(self.declare_parameter('pose_pid.kp', 1.5).value)
        self._pid_ki = float(self.declare_parameter('pose_pid.ki', 0.1).value)
        start_enabled = bool(self.declare_parameter(
            'start_enabled', True).value)
        trajectory_controller_name = str(self.declare_parameter(
            'trajectory_controller_name',
            'joint_trajectory_controller').value)
        self._pid_kd = float(self.declare_parameter('pose_pid.kd', 0.05).value)
        self._pid_limit = float(self.declare_parameter(
            'pose_pid.integral_limit', 0.5).value)
        if (self._control_frequency <= 0.0 or self._command_timeout <= 0.0 or
                self._velocity_ramp_rate <= 0.0 or
                self._stop_ramp_rate <= 0.0 or
                self._initial_pose_duration <= 0.0):
            raise ValueError(
                'control_frequency, command_timeout, ramp rates, and '
                'initial_pose_duration must be positive')

        geometry = RobotGeometry(**geometry_values)
        self._kinematics = QuadrupedKinematics(geometry)
        self._gait = GaitGenerator(geometry, GaitParameters(**gait_values))
        self._command = Twist()
        self._smoothed_velocity = [0.0, 0.0, 0.0]
        self._body_rpy = (0.0, 0.0, 0.0)
        self._body_translation = (0.0, 0.0, 0.0)
        self._imu_rpy = (0.0, 0.0, 0.0)
        self._imu_rates = (0.0, 0.0)
        self._integral = [0.0, 0.0]
        self._gait_enabled = start_enabled
        self._output_enabled = start_enabled
        self._initial_pose_pending = start_enabled
        self._initial_pose_deadline = None
        self._joint_positions: dict[str, float] = {}
        self._balance_enabled = False
        self._estop = False
        self._gait_values = gait_values
        self._contacts = {
            name: False for name in self._kinematics.nominal_foot_positions}
        self._last_command_time = self.get_clock().now()
        self._publisher = self.create_publisher(
            JointTrajectory,
            f'{trajectory_controller_name}/joint_trajectory', 10)
        self._subscription = self.create_subscription(
            Twist, 'cmd_vel', self._on_command, 10)
        self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, 10)
        self.create_subscription(PoseStamped, 'body_pose', self._on_pose, 10)
        self.create_subscription(
            Imu, 'imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Bool, 'estop', self._on_estop, 10)
        self.create_subscription(Bool, 'imu_auto', self._on_imu_auto, 10)
        self.create_subscription(
            Float64, 'gait/clearance_height',
            lambda message: self._set_gait_value(
                'clearance_height', message.data), 10)
        self.create_subscription(
            Float64, 'gait/penetration_depth',
            lambda message: self._set_gait_value(
                'penetration_depth', message.data), 10)
        self.create_subscription(
            Float64, 'gait/swing_duration',
            lambda message: self._set_gait_value(
                'swing_duration', message.data), 10)
        self.create_service(SetBool, 'gait/enable', self._set_gait_enabled)
        self.create_service(SetBool, 'balance/enable', self._set_balance_enabled)
        self._contact_subscriptions = [
            self.create_subscription(
                Bool,
                f'contacts/{name}',
                lambda message, leg=name: self._on_contact(leg, message),
                10,
            )
            for name in self._contacts
        ]
        self._timer = self.create_timer(
            1.0 / self._control_frequency, self._update)

    def _on_command(self, message: Twist) -> None:
        self._command = message
        self._last_command_time = self.get_clock().now()

    def _on_joint_state(self, message: JointState) -> None:
        self._joint_positions.update(zip(message.name, message.position))

    def _on_contact(self, leg_name: str, message: Bool) -> None:
        self._contacts[leg_name] = message.data

    def _on_pose(self, message: PoseStamped) -> None:
        pose = message.pose
        self._body_rpy = quaternion_to_rpy(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        self._body_translation = (
            pose.position.x, pose.position.y, pose.position.z)

    def _on_imu(self, message: Imu) -> None:
        orientation = message.orientation
        self._imu_rpy = quaternion_to_rpy(
            orientation.x, orientation.y, orientation.z, orientation.w)
        self._imu_rates = (
            message.angular_velocity.x, message.angular_velocity.y)

    def _on_estop(self, message: Bool) -> None:
        self._estop = message.data

    def _on_imu_auto(self, message: Bool) -> None:
        self._balance_enabled = message.data

    def _set_gait_value(self, name: str, value: float) -> None:
        values = dict(self._gait_values)
        values[name] = float(value)
        if name == 'swing_duration':
            values['stance_duration'] = min(
                values['stance_duration'], 1.3 * values[name])
        try:
            parameters = GaitParameters(**values)
        except ValueError as error:
            self.get_logger().error(f'Rejected gait adjustment: {error}')
            return
        phase = self._gait.phase
        self._gait_values = values
        self._gait = GaitGenerator(self._kinematics.geometry, parameters)
        self._gait.reset(phase)

    def _set_gait_enabled(self, request, response):
        output_was_disabled = not self._output_enabled
        self._gait_enabled = request.data
        self._output_enabled = True
        if output_was_disabled:
            self._initial_pose_pending = True
        self._smoothed_velocity = [0.0, 0.0, 0.0]
        if request.data:
            self._body_rpy = (0.0, 0.0, 0.0)
            self._body_translation = (0.0, 0.0, 0.0)
        response.success = True
        response.message = 'stepping' if request.data else 'viewing'
        return response

    def _set_balance_enabled(self, request, response):
        self._balance_enabled = request.data
        self._integral = [0.0, 0.0]
        response.success = True
        response.message = (
            'IMU balance enabled' if request.data else 'IMU balance disabled')
        return response

    def _controlled_body_rpy(self, dt: float) -> tuple[float, float, float]:
        if not self._balance_enabled:
            self._integral = [0.0, 0.0]
            return self._body_rpy
        errors = (
            self._body_rpy[0] - self._imu_rpy[0],
            self._body_rpy[1] - self._imu_rpy[1],
        )
        for index, error in enumerate(errors):
            self._integral[index] = max(
                -self._pid_limit,
                min(self._pid_limit, self._integral[index] + error * dt))
        return (
            -(self._pid_kp * errors[0] + self._pid_ki * self._integral[0]
              - self._pid_kd * self._imu_rates[0]),
            -(self._pid_kp * errors[1] + self._pid_ki * self._integral[1]
              - self._pid_kd * self._imu_rates[1]),
            self._body_rpy[2],
        )

    def _update(self) -> None:
        if not self._output_enabled:
            return
        now = self.get_clock().now()
        if (self._initial_pose_deadline is not None and
                now < self._initial_pose_deadline):
            return
        self._initial_pose_deadline = None
        command_age = (now - self._last_command_time).nanoseconds / 1.0e9
        if self._estop:
            self._smoothed_velocity = [0.0, 0.0, 0.0]
            return
        immediate_stop = command_age > self._command_timeout
        if immediate_stop:
            self._smoothed_velocity = [0.0, 0.0, 0.0]
        else:
            targets = (
                self._command.linear.x,
                self._command.linear.y,
                self._command.angular.z,
            )
            stopping = all(abs(target) <= 1.0e-6 for target in targets)
            ramp_rate = (self._stop_ramp_rate if stopping
                         else self._velocity_ramp_rate)
            dt = 1.0 / self._control_frequency
            self._smoothed_velocity = [
                ramp_value(current, target, ramp_rate, dt)
                for current, target in zip(self._smoothed_velocity, targets)
            ]
        velocity = tuple(self._smoothed_velocity[:2])
        yaw_rate = self._smoothed_velocity[2]

        try:
            if self._gait_enabled:
                feet = self._gait.update(
                    1.0 / self._control_frequency,
                    velocity,
                    yaw_rate,
                    self._contacts,
                    complete_cycle_on_stop=not immediate_stop,
                )
            else:
                feet = self._gait.update(
                    1.0 / self._control_frequency, (0.0, 0.0), 0.0,
                    self._contacts, complete_cycle_on_stop=False)
            positions = self._kinematics.inverse(
                self._controlled_body_rpy(1.0 / self._control_frequency),
                self._body_translation,
                feet,
            )
        except (ValueError, UnreachableTargetError) as error:
            self.get_logger().error(f'Rejected gait command: {error}')
            return

        trajectory = JointTrajectory()
        trajectory.header.stamp = now.to_msg()
        trajectory.joint_names = list(self._kinematics.joint_names)
        duration = (self._initial_pose_duration if self._initial_pose_pending
                    else 1.0 / self._control_frequency)
        point = JointTrajectoryPoint()
        point.positions = positions.reshape(-1).tolist()
        seconds = int(duration)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int((duration - seconds) * 1.0e9)
        if self._initial_pose_pending:
            if not all(name in self._joint_positions
                       for name in trajectory.joint_names):
                return
            start = JointTrajectoryPoint()
            start.positions = [
                self._joint_positions[name] for name in trajectory.joint_names]
            trajectory.points = [start]
            if self._staged_initial_pose:
                staged_targets = staged_joint_targets(
                    list(start.positions), list(point.positions))
                for stage, target in enumerate(staged_targets, start=1):
                    staged_point = JointTrajectoryPoint()
                    staged_point.positions = target
                    stage_time = stage * self._initial_pose_duration
                    staged_point.time_from_start.sec = int(stage_time)
                    staged_point.time_from_start.nanosec = int(
                        (stage_time - int(stage_time)) * 1.0e9)
                    trajectory.points.append(staged_point)
                duration *= len(staged_targets)
            else:
                trajectory.points.append(point)
        else:
            trajectory.points = [point]
        self._publisher.publish(trajectory)
        if self._initial_pose_pending:
            self._initial_pose_pending = False
            self._initial_pose_deadline = (
                now + Duration(seconds=duration))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GaitController()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
