// Read-only single-motor diagnostic CLI. Never sends Set_Axis_State,
// Set_Controller_Mode, or any Set_Input_* command -- it only requests
// telemetry, so it is safe to run for wiring/CAN-ID/bus verification before
// a motor has been calibrated or its safe limits confirmed
// (docs/packages/quattro_hardware.md section 6, step 2).

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "gim6010_driver/motor_manager.hpp"

namespace
{

struct Options
{
  std::string interface{"can0"};
  int node_id{0};
};

bool parse_args(int argc, char ** argv, Options & options)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--interface" && i + 1 < argc) {
      options.interface = argv[++i];
    } else if (arg == "--node-id" && i + 1 < argc) {
      options.node_id = std::atoi(argv[++i]);
    } else {
      std::fprintf(stderr, "Unknown or incomplete argument: %s\n", arg.c_str());
      return false;
    }
  }
  if (options.node_id < 0 || options.node_id > gim6010_driver::kMaxNodeId) {
    std::fprintf(stderr, "--node-id must be within [0, %u]\n", gim6010_driver::kMaxNodeId);
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  if (!parse_args(argc, argv, options)) {
    std::fprintf(
      stderr, "Usage: gim6010_diagnostic --interface <can0|can1> --node-id <0-63>\n");
    return 1;
  }

  const uint8_t node_id = static_cast<uint8_t>(options.node_id);
  gim6010_driver::MotorManager manager(
    {options.interface}, {gim6010_driver::MotorRoute{node_id, options.interface}});

  if (!manager.open()) {
    std::fprintf(stderr, "Failed to open %s\n", options.interface.c_str());
    return 1;
  }

  std::printf(
    "gim6010_diagnostic: interface=%s node_id=%u (read-only, no commands sent)\n",
    options.interface.c_str(), node_id);

  while (true) {
    manager.request_encoder_estimate(node_id);
    manager.request_bus_voltage_current(node_id);
    manager.request_get_error(node_id);

    // Give the bus time to answer before draining, since these are
    // request/response pairs rather than a background poll loop.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    manager.poll();

    const auto * motor = manager.motor(node_id);
    const auto heartbeat = motor->last_heartbeat();
    const auto encoder = motor->last_encoder_estimate();
    const auto bus = motor->last_bus_voltage_current();
    const auto error = motor->last_error();

    std::printf(
      "heartbeat=%s encoder=%s bus=%s error=%s bus_state=%d\n",
      heartbeat ? "OK" : "none",
      encoder ? "OK" : "none",
      bus ? "OK" : "none",
      error ? "OK" : "none",
      static_cast<int>(manager.bus_state(options.interface)));
    if (encoder) {
      std::printf(
        "  position_rev=%.4f velocity_rev_s=%.4f\n", encoder->position_rev,
        encoder->velocity_rev_s);
    }
    if (bus) {
      std::printf("  bus_voltage_V=%.2f bus_current_A=%.2f\n", bus->bus_voltage_V, bus->bus_current_A);
    }
    if (error && (error->active_errors != 0 || error->disarm_reason != 0)) {
      std::printf(
        "  active_errors=0x%08X disarm_reason=0x%08X\n", error->active_errors,
        error->disarm_reason);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(180));
  }
}
