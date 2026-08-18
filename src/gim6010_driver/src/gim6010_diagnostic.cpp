#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "gim6010_driver/motor_manager.hpp"

namespace
{
void pollUntil(
  gim6010_driver::MotorManager & manager, std::chrono::steady_clock::time_point deadline)
{
  while (std::chrono::steady_clock::now() < deadline) {
    manager.poll(std::chrono::milliseconds{20});
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: gim6010_diagnostic CAN_INTERFACE NODE_ID\n"
              << "Read-only: does not configure, enable, clear errors, or save settings.\n";
    return 2;
  }
  try {
    const long parsed_id = std::stol(argv[2]);
    if (parsed_id < 0 || parsed_id > gim6010_driver::Gim6010Motor::kMaxNodeId) {
      throw std::invalid_argument("NODE_ID must be 0..63");
    }
    gim6010_driver::MotorManager manager(argv[1]);
    const auto node_id = static_cast<std::uint8_t>(parsed_id);
    manager.addMotor(node_id);
    auto & motor = manager.motor(node_id);

    motor.requestEncoderEstimates();
    motor.requestIq();
    motor.requestBusVoltageCurrent();
    pollUntil(manager, std::chrono::steady_clock::now() + std::chrono::seconds{1});
    for (const auto type : {gim6010_driver::ErrorType::kMotor,
      gim6010_driver::ErrorType::kEncoder, gim6010_driver::ErrorType::kController,
      gim6010_driver::ErrorType::kSystem})
    {
      motor.requestError(type);
      pollUntil(manager, std::chrono::steady_clock::now() + std::chrono::milliseconds{100});
    }

    std::cout << "interface=" << argv[1] << " node_id=" << parsed_id << '\n';
    if (motor.hasHeartbeat()) {
      const auto & heartbeat = motor.heartbeat();
      std::cout << "heartbeat axis_state=" << static_cast<unsigned>(heartbeat.axis_state)
                << " axis_error=0x" << std::hex << heartbeat.axis_error
                << " flags=0x" << static_cast<unsigned>(heartbeat.flags) << std::dec << '\n';
    } else {
      std::cout << "heartbeat=unavailable\n";
    }
    if (motor.hasEncoderEstimates()) {
      const auto & feedback = motor.encoderEstimates();
      std::cout << "output_position_rad=" << feedback.position
                << " output_velocity_rad_s=" << feedback.velocity << '\n';
    } else {
      std::cout << "encoder_estimates=unavailable\n";
    }
    if (motor.hasBusVoltageCurrent()) {
      const auto & bus = motor.busVoltageCurrent();
      std::cout << "bus_voltage_v=" << bus.voltage << " bus_current_a=" << bus.current << '\n';
    }
    if (motor.hasIq()) {
      const auto & iq = motor.iq();
      std::cout << "iq_setpoint_a=" << iq.setpoint << " iq_measured_a=" << iq.measured << '\n';
    } else {
      std::cout << "iq=unavailable\n";
    }
    for (const auto type : {gim6010_driver::ErrorType::kMotor,
      gim6010_driver::ErrorType::kEncoder, gim6010_driver::ErrorType::kController,
      gim6010_driver::ErrorType::kSystem})
    {
      if (motor.hasError(type)) {
        std::cout << "error_type_" << static_cast<unsigned>(type) << "=0x" << std::hex
                  << motor.error(type) << std::dec << '\n';
      }
    }
    const auto & can = manager.canErrorStatus();
    std::cout << "can_error_frames=" << can.total_frames
              << " warning=" << can.warning_frames << " passive=" << can.passive_frames
              << " bus_off=" << can.bus_off_frames << " txerr="
              << static_cast<unsigned>(can.tx_error_counter) << " rxerr="
              << static_cast<unsigned>(can.rx_error_counter) << '\n';
    return motor.hasHeartbeat() && motor.hasEncoderEstimates() ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "gim6010_diagnostic: " << error.what() << '\n';
    return 1;
  }
}
