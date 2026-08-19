// Read-only single-motor diagnostic CLI. Never changes control mode, gain,
// limits, or enable state, and never clears errors or saves configuration --
// see docs/hardware_bringup.md for how this fits into the real-robot fault
// diagnosis procedure.
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "gim6010_driver/motor_manager.hpp"

namespace
{
void printUsage(const char * program)
{
  std::cerr << "usage: " << program << " <can_interface> <node_id>\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    printUsage(argv[0]);
    return EXIT_FAILURE;
  }
  const std::string interface_name = argv[1];
  int node_id_value = 0;
  try {
    node_id_value = std::stoi(argv[2]);
  } catch (const std::exception &) {
    printUsage(argv[0]);
    return EXIT_FAILURE;
  }
  if (node_id_value < 0 || node_id_value > gim6010_driver::Gim6010Motor::kMaxNodeId) {
    std::cerr << "node_id must be in [0, "
              << static_cast<int>(gim6010_driver::Gim6010Motor::kMaxNodeId) << "]\n";
    return EXIT_FAILURE;
  }
  const auto node_id = static_cast<std::uint8_t>(node_id_value);

  try {
    gim6010_driver::MotorManager manager(interface_name);
    manager.addMotor(node_id);
    auto & motor = manager.motor(node_id);

    motor.requestEncoderEstimates();
    motor.requestEncoderCount();
    motor.requestIq();
    motor.requestBusVoltageCurrent();
    constexpr std::array<gim6010_driver::ErrorType, 4> error_types{{
      gim6010_driver::ErrorType::kMotor, gim6010_driver::ErrorType::kEncoder,
      gim6010_driver::ErrorType::kController, gim6010_driver::ErrorType::kSystem}};
    for (const auto type : error_types) {
      motor.requestError(type);
    }

    // Bounded to 500 ms total regardless of bus traffic: each poll() call
    // waits at most the *remaining* budget, and the loop condition is
    // rechecked after every single frame. A busy bus with many other nodes
    // broadcasting heartbeats must not turn this into an unbounded drain --
    // `while (manager.poll(10ms)) {}` would never see a 10ms gap and never
    // return on a live multi-motor bus.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{500};
    while (true) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - now);
      manager.poll(std::min(remaining, std::chrono::milliseconds{10}));
    }

    std::cout << "interface: " << interface_name
              << "  node_id: " << static_cast<int>(node_id) << "\n";
    if (motor.hasHeartbeat()) {
      const auto & heartbeat = motor.heartbeat();
      std::cout << "heartbeat: axis_state=" << static_cast<unsigned>(heartbeat.axis_state)
                << " axis_error=0x" << std::hex << heartbeat.axis_error << std::dec
                << " flags=0x" << std::hex << static_cast<unsigned>(heartbeat.flags) << std::dec
                << " life=" << static_cast<unsigned>(heartbeat.life)
                << " missed=" << motor.missedHeartbeats() << "\n";
    } else {
      std::cout << "heartbeat: none received\n";
    }
    if (motor.hasEncoderEstimates()) {
      const auto & estimates = motor.encoderEstimates();
      std::cout << "output position: " << estimates.output_position_rad
                << " rad  velocity: " << estimates.output_velocity_rad_s << " rad/s\n";
    } else {
      std::cout << "encoder estimates: none received\n";
    }
    if (motor.hasEncoderCount()) {
      std::cout << "encoder count (diagnostic only, not a second sensor): shadow_count="
                << motor.encoderCount().shadow_count
                << " count_in_cpr=" << motor.encoderCount().count_in_cpr << "\n";
    }
    if (motor.hasIq()) {
      std::cout << "iq setpoint: " << motor.iq().setpoint
                << " A  measured: " << motor.iq().measured << " A\n";
    }
    if (motor.hasBusVoltageCurrent()) {
      std::cout << "bus voltage: " << motor.busVoltageCurrent().voltage
                << " V  current: " << motor.busVoltageCurrent().current << " A\n";
    }
    for (const auto type : error_types) {
      if (motor.hasError(type)) {
        std::cout << "error[" << static_cast<int>(type) << "]: 0x" << std::hex
                  << motor.error(type) << std::dec << "\n";
      }
    }
    const auto & can_status = manager.canErrorStatus();
    std::cout << "CAN: total_error_frames=" << can_status.total_frames
              << " bus_off=" << can_status.bus_off_frames
              << " passive=" << can_status.passive_frames
              << " warning=" << can_status.warning_frames
              << " tx_err_ctr=" << can_status.tx_error_counter
              << " rx_err_ctr=" << can_status.rx_error_counter
              << " rx_dropped=" << can_status.rx_dropped_frames << "\n";
  } catch (const std::exception & error) {
    std::cerr << "gim6010_diagnostic failed: " << error.what() << "\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
