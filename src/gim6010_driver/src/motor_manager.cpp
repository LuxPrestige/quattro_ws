#include "gim6010_driver/motor_manager.hpp"

#include <algorithm>
#include <chrono>
#include <set>
#include <stdexcept>

namespace gim6010_driver
{

MotorManager::MotorManager(
  std::vector<std::string> bus_interfaces, std::vector<MotorRoute> routes)
: MotorManager(
    std::move(bus_interfaces), std::move(routes),
    [](const std::string & interface_name) -> std::unique_ptr<CanSocketInterface> {
      return std::make_unique<CanSocket>(interface_name);
    })
{
}

MotorManager::MotorManager(
  std::vector<std::string> bus_interfaces, std::vector<MotorRoute> routes,
  SocketFactory socket_factory)
{
  if (!socket_factory) {
    throw std::invalid_argument("MotorManager requires a non-empty socket factory");
  }
  if (bus_interfaces.empty()) {
    throw std::invalid_argument("MotorManager requires at least one CAN bus interface");
  }

  std::set<std::string> known_buses;
  for (const auto & interface_name : bus_interfaces) {
    if (interface_name.empty()) {
      throw std::invalid_argument("MotorManager bus interface name must not be empty");
    }
    if (!known_buses.insert(interface_name).second) {
      throw std::invalid_argument("MotorManager bus interface listed more than once: " + interface_name);
    }
    auto socket = socket_factory(interface_name);
    if (!socket) {
      throw std::invalid_argument(
        "MotorManager socket factory returned nothing for bus: " + interface_name);
    }
    sockets_.emplace(interface_name, std::move(socket));
  }

  for (const auto & route : routes) {
    if (route.node_id > kMaxNodeId) {
      throw std::invalid_argument("MotorManager route node_id exceeds kMaxNodeId");
    }
    if (route.bus.empty() || known_buses.find(route.bus) == known_buses.end()) {
      throw std::invalid_argument("MotorManager route references unknown bus: " + route.bus);
    }
    if (!node_to_bus_.emplace(route.node_id, route.bus).second) {
      throw std::invalid_argument("MotorManager has a duplicate node_id in routes");
    }
    motors_.emplace(route.node_id, Gim6010Motor(route.node_id));
  }
}

bool MotorManager::open()
{
  std::vector<std::string> opened;
  for (auto & [interface_name, socket] : sockets_) {
    if (!socket->open()) {
      for (const auto & already_opened : opened) {
        sockets_.at(already_opened)->close();
      }
      return false;
    }
    opened.push_back(interface_name);
  }
  return true;
}

void MotorManager::close()
{
  for (auto & [interface_name, socket] : sockets_) {
    socket->close();
  }
}

void MotorManager::dispatch(const CanFrame & frame)
{
  const uint8_t node_id = node_id_from_arbitration_id(frame.id);
  const auto motor_it = motors_.find(node_id);
  if (motor_it == motors_.end()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto cmd = static_cast<CommandId>(cmd_id_from_arbitration_id(frame.id));
  switch (cmd) {
    case CommandId::kHeartbeat:
      motor_it->second.on_heartbeat(decode_heartbeat(frame), now);
      break;
    case CommandId::kGetError:
      motor_it->second.on_error_response(decode_get_error_response(frame), now);
      break;
    case CommandId::kMitControl:
      motor_it->second.on_mit_feedback(decode_mit_feedback(frame), now);
      break;
    case CommandId::kGetEncoderEstimates:
      motor_it->second.on_encoder_estimate(decode_encoder_estimates(frame), now);
      break;
    case CommandId::kGetEncoderCount:
      motor_it->second.on_encoder_count(decode_encoder_count(frame), now);
      break;
    case CommandId::kGetBusVoltageCurrent:
      motor_it->second.on_bus_voltage_current(decode_bus_voltage_current(frame), now);
      break;
    case CommandId::kGetTorques:
      motor_it->second.on_torques(decode_torques(frame), now);
      break;
    case CommandId::kTxSdo:
      motor_it->second.on_sdo_response(decode_txsdo(frame), now);
      break;
    default:
      // Encode-only commands (Estop, Set_Axis_State, ...) are not decoded
      // as feedback; anything the motor echoes back for them is ignored.
      break;
  }
}

void MotorManager::poll()
{
  for (auto & [interface_name, socket] : sockets_) {
    while (const auto frame = socket->receive_nonblocking()) {
      dispatch(*frame);
    }
  }
}

bool MotorManager::send_to(uint8_t node_id, const CanFrame & frame)
{
  const auto route_it = node_to_bus_.find(node_id);
  if (route_it == node_to_bus_.end()) {
    return false;
  }
  return sockets_.at(route_it->second)->send(frame);
}

bool MotorManager::send_estop(uint8_t node_id) { return send_to(node_id, encode_estop(node_id)); }

bool MotorManager::send_set_axis_node_id(uint8_t node_id, uint8_t new_node_id)
{
  return send_to(node_id, encode_set_axis_node_id(node_id, new_node_id));
}

bool MotorManager::send_set_axis_state(uint8_t node_id, AxisState state)
{
  return send_to(node_id, encode_set_axis_state(node_id, state));
}

bool MotorManager::send_set_controller_mode(
  uint8_t node_id, ControlMode control_mode, InputMode input_mode)
{
  return send_to(node_id, encode_set_controller_mode(node_id, control_mode, input_mode));
}

bool MotorManager::send_set_input_pos(uint8_t node_id, const SetInputPosCommand & command)
{
  const auto frame = encode_set_input_pos(node_id, command);
  if (!frame) {
    return false;
  }
  return send_to(node_id, *frame);
}

bool MotorManager::send_set_input_vel(uint8_t node_id, float velocity_rev_s, float torque_ff_Nm)
{
  return send_to(node_id, encode_set_input_vel(node_id, velocity_rev_s, torque_ff_Nm));
}

bool MotorManager::send_set_input_torque(uint8_t node_id, float torque_Nm)
{
  return send_to(node_id, encode_set_input_torque(node_id, torque_Nm));
}

bool MotorManager::send_mit_command(uint8_t node_id, const MitCommand & command)
{
  const auto frame = encode_mit_command(node_id, command);
  if (!frame) {
    return false;
  }
  return send_to(node_id, *frame);
}

bool MotorManager::send_set_limits(
  uint8_t node_id, float velocity_limit_rev_s, float current_limit_A)
{
  return send_to(node_id, encode_set_limits(node_id, velocity_limit_rev_s, current_limit_A));
}

bool MotorManager::send_set_pos_gain(uint8_t node_id, float pos_gain)
{
  return send_to(node_id, encode_set_pos_gain(node_id, pos_gain));
}

bool MotorManager::send_set_vel_gains(uint8_t node_id, float vel_gain, float vel_integrator_gain)
{
  return send_to(node_id, encode_set_vel_gains(node_id, vel_gain, vel_integrator_gain));
}

bool MotorManager::send_clear_errors(uint8_t node_id)
{
  return send_to(node_id, encode_clear_errors(node_id));
}

bool MotorManager::send_save_configuration(uint8_t node_id)
{
  return send_to(node_id, encode_save_configuration(node_id));
}

bool MotorManager::send_set_traj_vel_limit(uint8_t node_id, float traj_vel_limit_rev_s)
{
  return send_to(node_id, encode_set_traj_vel_limit(node_id, traj_vel_limit_rev_s));
}

bool MotorManager::send_set_traj_accel_limits(
  uint8_t node_id, float traj_accel_limit_rev_s2, float traj_decel_limit_rev_s2)
{
  return send_to(
    node_id,
    encode_set_traj_accel_limits(node_id, traj_accel_limit_rev_s2, traj_decel_limit_rev_s2));
}

bool MotorManager::send_set_traj_inertia(uint8_t node_id, float traj_inertia)
{
  return send_to(node_id, encode_set_traj_inertia(node_id, traj_inertia));
}

bool MotorManager::send_disable_can(uint8_t node_id)
{
  return send_to(node_id, encode_disable_can(node_id));
}

bool MotorManager::request_sdo_read(uint8_t node_id, uint16_t endpoint_id)
{
  return send_to(node_id, encode_rxsdo(node_id, SdoOpcode::kRead, endpoint_id));
}

bool MotorManager::send_sdo_write(uint8_t node_id, uint16_t endpoint_id, SdoValue value)
{
  return send_to(node_id, encode_rxsdo(node_id, SdoOpcode::kWrite, endpoint_id, value));
}

bool MotorManager::request_get_error(uint8_t node_id)
{
  return send_to(node_id, encode_get_error_request(node_id));
}

bool MotorManager::request_encoder_estimate(uint8_t node_id)
{
  return send_to(node_id, encode_get_encoder_estimates_request(node_id));
}

bool MotorManager::request_encoder_count(uint8_t node_id)
{
  return send_to(node_id, encode_get_encoder_count_request(node_id));
}

bool MotorManager::request_bus_voltage_current(uint8_t node_id)
{
  return send_to(node_id, encode_get_bus_voltage_current_request(node_id));
}

bool MotorManager::request_torques(uint8_t node_id)
{
  return send_to(node_id, encode_get_torques_request(node_id));
}

Gim6010Motor * MotorManager::motor(uint8_t node_id)
{
  const auto it = motors_.find(node_id);
  return it == motors_.end() ? nullptr : &it->second;
}

const Gim6010Motor * MotorManager::motor(uint8_t node_id) const
{
  const auto it = motors_.find(node_id);
  return it == motors_.end() ? nullptr : &it->second;
}

std::vector<uint8_t> MotorManager::node_ids() const
{
  std::vector<uint8_t> ids;
  ids.reserve(motors_.size());
  for (const auto & [node_id, motor] : motors_) {
    ids.push_back(node_id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

CanBusState MotorManager::bus_state(const std::string & bus) const
{
  const auto it = sockets_.find(bus);
  return it == sockets_.end() ? CanBusState::kActive : it->second->bus_state();
}

}  // namespace gim6010_driver
