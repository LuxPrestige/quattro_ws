#include <gtest/gtest.h>

#include <linux/can.h>
#include <linux/can/error.h>

#include "gim6010_driver/can_error.hpp"

namespace gim6010_driver
{
namespace
{

TEST(CanError, DecodesWarningState)
{
  struct can_frame raw{};
  raw.can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
  raw.can_dlc = 8;
  raw.data[1] = CAN_ERR_CRTL_TX_WARNING;
  raw.data[6] = 96;
  raw.data[7] = 10;

  const CanBusError error = decode_error_frame(raw);
  EXPECT_TRUE(error.error_warning);
  EXPECT_FALSE(error.error_passive);
  EXPECT_FALSE(error.bus_off);
  EXPECT_EQ(error.tx_error_counter, 96);
  EXPECT_EQ(error.rx_error_counter, 10);
  EXPECT_EQ(bus_state_from_error(error), CanBusState::kWarning);
}

TEST(CanError, DecodesPassiveState)
{
  struct can_frame raw{};
  raw.can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
  raw.data[1] = CAN_ERR_CRTL_RX_PASSIVE;

  const CanBusError error = decode_error_frame(raw);
  EXPECT_TRUE(error.error_passive);
  EXPECT_EQ(bus_state_from_error(error), CanBusState::kPassive);
}

TEST(CanError, DecodesBusOff)
{
  struct can_frame raw{};
  raw.can_id = CAN_ERR_FLAG | CAN_ERR_BUSOFF;

  const CanBusError error = decode_error_frame(raw);
  EXPECT_TRUE(error.bus_off);
  EXPECT_EQ(bus_state_from_error(error), CanBusState::kBusOff);
}

TEST(CanError, NoFlagsMeansActive)
{
  CanBusError error;
  EXPECT_EQ(bus_state_from_error(error), CanBusState::kActive);
}

TEST(CanError, BusOffTakesPriorityOverPassiveAndWarning)
{
  CanBusError error;
  error.bus_off = true;
  error.error_passive = true;
  error.error_warning = true;
  EXPECT_EQ(bus_state_from_error(error), CanBusState::kBusOff);
}

}  // namespace
}  // namespace gim6010_driver
