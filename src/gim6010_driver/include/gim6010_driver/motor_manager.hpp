#ifndef GIM6010_DRIVER__MOTOR_MANAGER_HPP_
#define GIM6010_DRIVER__MOTOR_MANAGER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "gim6010_driver/can_simple_messages.hpp"
#include "gim6010_driver/can_socket.hpp"
#include "gim6010_driver/can_socket_interface.hpp"
#include "gim6010_driver/gim6010_motor.hpp"
#include "gim6010_driver/mit_protocol.hpp"
#include "gim6010_driver/types.hpp"

namespace gim6010_driver
{

// Owns one CanSocketInterface per configured bus and one Gim6010Motor per
// configured node ID, and is the only class in this driver that touches multiple
// motors or buses at once. Not specific to any particular joint layout or
// bus count -- pass whatever set of interface names and node-ID routes the
// calling project actually has.
//
// poll() drains every bus non-blocking and dispatches decoded frames to the
// matching Gim6010Motor; no background thread is started. Callers (e.g. a
// ros2_control read() running at a fixed update rate) are expected to call
// poll() once per cycle themselves.
class MotorManager
{
public:
  // Builds one transport per configured bus. The default builds a real
  // CanSocket; a test passes its own so the whole stack above this line
  // (routing, dispatch, and any lifecycle logic built on it) runs
  // unmodified against an in-memory bus.
  using SocketFactory =
    std::function<std::unique_ptr<CanSocketInterface>(const std::string & interface_name)>;

  // Throws std::invalid_argument for malformed static configuration
  // (duplicate node_id, node_id > kMaxNodeId, a route referencing a bus not
  // present in `bus_interfaces`, or an empty bus/interface name) -- these
  // are programming/config errors, not runtime I/O conditions.
  MotorManager(std::vector<std::string> bus_interfaces, std::vector<MotorRoute> routes);
  // Same contract, plus: throws std::invalid_argument if `socket_factory`
  // is empty or returns null for any configured bus.
  MotorManager(
    std::vector<std::string> bus_interfaces, std::vector<MotorRoute> routes,
    SocketFactory socket_factory);

  // Opens every configured bus. Returns false if any bus fails to open;
  // already-opened buses from this call are closed again before returning
  // so a partial failure never leaves some buses open and others not.
  bool open();
  void close();

  // Drains every bus (non-blocking) and updates motor state. Call once per
  // control cycle; does not block and starts no thread.
  void poll();

  // Low-level send on a specific motor's route. Returns false if node_id is
  // unknown or the send itself fails.
  bool send_to(uint8_t node_id, const CanFrame & frame);

  bool send_estop(uint8_t node_id);
  bool send_set_axis_node_id(uint8_t node_id, uint8_t new_node_id);
  bool send_set_axis_state(uint8_t node_id, AxisState state);
  bool send_set_controller_mode(uint8_t node_id, ControlMode control_mode, InputMode input_mode);
  // Returns false both when node_id is unknown and when the command's
  // feed-forward fields do not fit the wire format -- callers distinguish
  // the two only if they need to (typically both mean "do not send").
  bool send_set_input_pos(uint8_t node_id, const SetInputPosCommand & command);
  bool send_set_input_vel(uint8_t node_id, float velocity_rev_s, float torque_ff_Nm);
  bool send_set_input_torque(uint8_t node_id, float torque_Nm);
  bool send_mit_command(uint8_t node_id, const MitCommand & command);
  bool send_set_limits(uint8_t node_id, float velocity_limit_rev_s, float current_limit_A);
  bool send_set_pos_gain(uint8_t node_id, float pos_gain);
  bool send_set_vel_gains(uint8_t node_id, float vel_gain, float vel_integrator_gain);
  bool send_clear_errors(uint8_t node_id);
  bool send_save_configuration(uint8_t node_id);
  bool send_set_traj_vel_limit(uint8_t node_id, float traj_vel_limit_rev_s);
  bool send_set_traj_accel_limits(
    uint8_t node_id, float traj_accel_limit_rev_s2, float traj_decel_limit_rev_s2);
  bool send_set_traj_inertia(uint8_t node_id, float traj_inertia);
  // Effectively irreversible from software on most firmware -- see
  // can_simple_messages.hpp's encode_disable_can documentation.
  bool send_disable_can(uint8_t node_id);

  // Generic parameter access (see can_simple_messages.hpp for why endpoint
  // IDs are not named here). Response arrives asynchronously through
  // poll()/dispatch() and is read back via motor(node_id)->last_sdo_response().
  bool request_sdo_read(uint8_t node_id, uint16_t endpoint_id);
  bool send_sdo_write(uint8_t node_id, uint16_t endpoint_id, SdoValue value);

  bool request_get_error(uint8_t node_id);
  // Kept as a general driver/diagnostic capability. Motors wired into
  // Quattro are configured to broadcast Get_Encoder_Estimates on their own
  // (~10 ms), so nothing on the normal control path requests one -- see
  // docs/packages/gim6010_driver.md section 3.
  bool request_encoder_estimate(uint8_t node_id);
  bool request_encoder_count(uint8_t node_id);
  bool request_bus_voltage_current(uint8_t node_id);
  bool request_torques(uint8_t node_id);

  Gim6010Motor * motor(uint8_t node_id);
  const Gim6010Motor * motor(uint8_t node_id) const;
  std::vector<uint8_t> node_ids() const;

  CanBusState bus_state(const std::string & bus) const;

  // Exposed for tests: updates motor state from a frame as if it had just
  // been received, without touching any socket. Production code reaches
  // this only through poll().
  void dispatch(const CanFrame & frame);

private:
  std::unordered_map<std::string, std::unique_ptr<CanSocketInterface>> sockets_;
  std::unordered_map<uint8_t, std::string> node_to_bus_;
  std::unordered_map<uint8_t, Gim6010Motor> motors_;
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__MOTOR_MANAGER_HPP_
