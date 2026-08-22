"""
Explicit startup state machine for the real Quattro robot.

Owns the order in which the hardware component and the controllers are
brought up, and refuses to reach READY unless every step actually
succeeded. It sends no CAN traffic of its own: limits, gains, controller
mode, closed-loop entry, post-closed-loop encoder synchronization and safe
stop are all ``QuattroSystem``'s responsibility (see
``docs/packages/quattro_hardware.md``). This node only drives lifecycle
transitions through ``controller_manager`` services and verifies the result.

Replaces the previous ``OnProcessExit`` spawner chain in
``hardware.launch.py``: a chain of process exits cannot distinguish "the
spawner exited because it succeeded" from "it exited because it gave up",
and has nowhere to put the verification steps between them.

READY means:

* ``QuattroSystem`` active (so all 12 GIM6010-8 are in closed-loop control
  and synchronized to a post-closed-loop encoder reading)
* ``joint_state_broadcaster`` active and publishing 12 valid joints
* ``joint_trajectory_controller`` active
* gait OFF -- the robot holds position and is not walking
"""

from enum import Enum
import math

from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    ListHardwareComponents,
    LoadController,
    SetHardwareComponentState,
    SwitchController,
)
from lifecycle_msgs.msg import State
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

HARDWARE_COMPONENT = 'QuattroSystem'
JOINT_STATE_BROADCASTER = 'joint_state_broadcaster'
JOINT_TRAJECTORY_CONTROLLER = 'joint_trajectory_controller'


class BringupState(Enum):
    """Startup stages, in the order they are traversed."""

    WAIT_CONTROLLER_MANAGER = 'WAIT_CONTROLLER_MANAGER'
    CONFIGURE_HARDWARE = 'CONFIGURE_HARDWARE'
    ACTIVATE_HARDWARE = 'ACTIVATE_HARDWARE'
    VERIFY_HARDWARE = 'VERIFY_HARDWARE'
    START_JSB = 'START_JSB'
    VERIFY_JOINT_STATES = 'VERIFY_JOINT_STATES'
    START_JTC = 'START_JTC'
    READY = 'READY'
    FAULT = 'FAULT'


class BringupManager(Node):
    """Drives the Quattro hardware and controllers to READY, or to FAULT."""

    def __init__(self) -> None:
        super().__init__('bringup_manager')

        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('service_timeout', 30.0)
        self.declare_parameter('joint_state_timeout', 10.0)
        self.declare_parameter('expected_joints', 12)
        self.declare_parameter('shutdown_on_fault', True)

        prefix = self.get_parameter('controller_manager').value.rstrip('/')
        self._service_timeout = float(self.get_parameter('service_timeout').value)
        self._joint_state_timeout = float(
            self.get_parameter('joint_state_timeout').value)
        self._expected_joints = int(self.get_parameter('expected_joints').value)
        self.shutdown_on_fault = bool(
            self.get_parameter('shutdown_on_fault').value)

        self._set_hardware_state = self.create_client(
            SetHardwareComponentState, f'{prefix}/set_hardware_component_state')
        self._list_hardware = self.create_client(
            ListHardwareComponents, f'{prefix}/list_hardware_components')
        self._load_controller = self.create_client(
            LoadController, f'{prefix}/load_controller')
        self._configure_controller = self.create_client(
            ConfigureController, f'{prefix}/configure_controller')
        self._list_controllers = self.create_client(
            ListControllers, f'{prefix}/list_controllers')
        self._switch_controller = self.create_client(
            SwitchController, f'{prefix}/switch_controller')

        self._last_joint_state: JointState | None = None
        self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, 10)

        self.state = BringupState.WAIT_CONTROLLER_MANAGER

    # -- helpers ----------------------------------------------------------

    def _on_joint_state(self, message: JointState) -> None:
        self._last_joint_state = message

    def _call(self, client, request, timeout: float):
        """Call a service and spin until it answers. Returns None on failure."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error(
                f'{client.srv_name} did not answer within {timeout:.1f}s')
            return None
        return future.result()

    def _fail(self, reason: str) -> bool:
        self.state = BringupState.FAULT
        self.get_logger().error(f'BRINGUP FAULT: {reason}')
        # Deliberately no attempt to idle the motors from here. A hardware
        # fault already idles every motor inside QuattroSystem, and a
        # bringup-side failure (a controller that would not start, say) must
        # not be "fixed" by a second actor sending axis-state commands on a
        # bus QuattroSystem still owns.
        self.get_logger().error(
            'Robot is NOT ready. Inspect the controller_manager log above, '
            'then check: ros2 control list_hardware_components; '
            'ros2 control list_controllers')
        return False

    # -- states -----------------------------------------------------------

    def wait_for_controller_manager(self) -> bool:
        """Block until every controller_manager service this node uses exists."""
        self.get_logger().info('Waiting for controller_manager services...')
        for client in (
            self._set_hardware_state, self._list_hardware,
            self._load_controller, self._configure_controller,
            self._list_controllers, self._switch_controller,
        ):
            if not client.wait_for_service(timeout_sec=self._service_timeout):
                return self._fail(
                    f'controller_manager service {client.srv_name} never appeared')
        self.get_logger().info('controller_manager is up.')
        return True

    def _set_hardware_component_state(self, label: str, state_id: int) -> bool:
        request = SetHardwareComponentState.Request()
        request.name = HARDWARE_COMPONENT
        request.target_state.id = state_id
        request.target_state.label = label
        response = self._call(
            self._set_hardware_state, request, self._service_timeout)
        if response is None:
            return self._fail(
                f'set_hardware_component_state({label}) did not answer')
        if not response.ok:
            return self._fail(
                f'{HARDWARE_COMPONENT} refused the transition to {label} '
                f'(it is now "{response.state.label}"). This is the hardware '
                f'rejecting startup, not a bringup ordering problem -- the '
                f'reason is in the controller_manager log.')
        return True

    def configure_hardware(self) -> bool:
        """Configure QuattroSystem: CAN open, limits, gains, Position+PosFilter."""
        self.get_logger().info(
            f'Configuring {HARDWARE_COMPONENT} '
            '(CAN open, heartbeat check, limits, gains, '
            'Position Control + Pos Filter)...')
        return self._set_hardware_component_state(
            'inactive', State.PRIMARY_STATE_INACTIVE)

    def activate_hardware(self) -> bool:
        """Activate QuattroSystem: closed loop plus post-closed-loop encoder sync."""
        self.get_logger().info(
            f'Activating {HARDWARE_COMPONENT} '
            '(closed loop, then post-closed-loop encoder synchronization). '
            'The motors hold their current position; nothing moves.')
        return self._set_hardware_component_state(
            'active', State.PRIMARY_STATE_ACTIVE)

    def verify_hardware(self) -> bool:
        """Confirm the resource manager really reports QuattroSystem active."""
        response = self._call(
            self._list_hardware, ListHardwareComponents.Request(),
            self._service_timeout)
        if response is None:
            return self._fail('list_hardware_components did not answer')
        for component in response.component:
            if component.name != HARDWARE_COMPONENT:
                continue
            if component.state.id != State.PRIMARY_STATE_ACTIVE:
                return self._fail(
                    f'{HARDWARE_COMPONENT} is "{component.state.label}", not active')
            self.get_logger().info(
                f'{HARDWARE_COMPONENT} is active: 12 motors in closed loop, '
                'encoder synchronized.')
            return True
        return self._fail(
            f'{HARDWARE_COMPONENT} is not loaded by controller_manager')

    def _controller_state(self, name: str) -> str | None:
        response = self._call(
            self._list_controllers, ListControllers.Request(),
            self._service_timeout)
        if response is None:
            return None
        for controller in response.controller:
            if controller.name == name:
                return controller.state
        return ''

    def _start_controller(self, name: str) -> bool:
        """Load, configure and activate one controller, skipping done steps."""
        state = self._controller_state(name)
        if state is None:
            return self._fail(f'list_controllers did not answer for {name}')

        if state == '':
            response = self._call(
                self._load_controller, LoadController.Request(name=name),
                self._service_timeout)
            if response is None or not response.ok:
                return self._fail(f'failed to load controller {name}')
            state = 'unconfigured'

        if state == 'unconfigured':
            response = self._call(
                self._configure_controller,
                ConfigureController.Request(name=name), self._service_timeout)
            if response is None or not response.ok:
                return self._fail(f'failed to configure controller {name}')
            state = 'inactive'

        if state == 'active':
            self.get_logger().info(f'{name} is already active.')
            return True

        request = SwitchController.Request()
        request.activate_controllers = [name]
        # STRICT: a controller that did not actually start must fail bringup
        # rather than leave the robot in a half-started state that looks
        # ready.
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = False
        request.timeout.sec = int(self._service_timeout)
        response = self._call(
            self._switch_controller, request, self._service_timeout)
        if response is None or not response.ok:
            message = getattr(response, 'message', '') if response else ''
            return self._fail(f'failed to activate controller {name}: {message}')

        if self._controller_state(name) != 'active':
            return self._fail(f'{name} did not reach the active state')
        self.get_logger().info(f'{name} is active.')
        return True

    def start_joint_state_broadcaster(self) -> bool:
        """Start the broadcaster that publishes /joint_states."""
        self.get_logger().info(f'Starting {JOINT_STATE_BROADCASTER}...')
        return self._start_controller(JOINT_STATE_BROADCASTER)

    def verify_joint_states(self) -> bool:
        """Wait for a /joint_states message with 12 finite joint positions."""
        self.get_logger().info(
            'Waiting for valid /joint_states from the post-closed-loop '
            'encoders...')
        self._last_joint_state = None
        deadline = self.get_clock().now().nanoseconds + int(
            self._joint_state_timeout * 1e9)
        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            message = self._last_joint_state
            if message is None:
                continue
            if len(message.position) != self._expected_joints:
                # Keep waiting: an early message can legitimately be
                # incomplete while interfaces are still being exported.
                continue
            # NaN is how QuattroSystem::read() reports "no encoder value for
            # this joint", so a full-length message is not by itself proof
            # that every joint is actually reporting.
            if any(not math.isfinite(value) for value in message.position):
                continue
            self.get_logger().info(
                f'/joint_states is valid: {self._expected_joints} joints.')
            return True
        return self._fail(
            f'no /joint_states message with {self._expected_joints} finite '
            f'positions within {self._joint_state_timeout:.1f}s')

    def start_joint_trajectory_controller(self) -> bool:
        """Start the trajectory controller that owns the position commands."""
        self.get_logger().info(f'Starting {JOINT_TRAJECTORY_CONTROLLER}...')
        return self._start_controller(JOINT_TRAJECTORY_CONTROLLER)

    # -- driver -----------------------------------------------------------

    def run(self) -> bool:
        """Run the state machine to READY. Returns False if it hit FAULT."""
        steps = (
            (BringupState.WAIT_CONTROLLER_MANAGER, self.wait_for_controller_manager),
            (BringupState.CONFIGURE_HARDWARE, self.configure_hardware),
            (BringupState.ACTIVATE_HARDWARE, self.activate_hardware),
            (BringupState.VERIFY_HARDWARE, self.verify_hardware),
            (BringupState.START_JSB, self.start_joint_state_broadcaster),
            (BringupState.VERIFY_JOINT_STATES, self.verify_joint_states),
            (BringupState.START_JTC, self.start_joint_trajectory_controller),
        )
        for state, step in steps:
            self.state = state
            self.get_logger().info(f'--> {state.value}')
            if not step():
                return False

        self.state = BringupState.READY
        self.get_logger().info(
            '--> READY\n'
            f'  {HARDWARE_COMPONENT:<28} ACTIVE\n'
            '  GIM6010-8 x12                CLOSED LOOP\n'
            '  Encoder                      post-closed-loop synchronized\n'
            f'  {JOINT_STATE_BROADCASTER:<28} ACTIVE\n'
            f'  {JOINT_TRAJECTORY_CONTROLLER:<28} ACTIVE\n'
            '  Gait                         OFF\n'
            'The robot is holding position. READY is not walking: start the '
            'gait explicitly when you want it.')
        return True


def main(args=None) -> None:
    """Run the bringup state machine, then idle so the node stays inspectable."""
    rclpy.init(args=args)
    node = BringupManager()
    try:
        succeeded = node.run()
        if not succeeded and node.shutdown_on_fault:
            # Non-zero exit so the launch file can tear the stack down
            # instead of leaving a half-started robot running.
            raise SystemExit(1)
        # Stay alive after READY: the node's state is worth being able to
        # inspect, and shutting down here would look like a crash.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
