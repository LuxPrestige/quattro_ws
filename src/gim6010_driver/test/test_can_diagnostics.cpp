#include <gtest/gtest.h>

#include "gim6010_driver/can_diagnostics.hpp"
#include "gim6010_driver/can_error.hpp"

namespace
{
// linux/can/error.h class flags used to build synthetic error frames.
constexpr std::uint32_t kErrCrtl = 0x00000004U;
constexpr std::uint32_t kErrAck = 0x00000020U;
constexpr std::uint32_t kErrBusoff = 0x00000040U;
constexpr std::uint32_t kErrCnt = 0x00000200U;
constexpr std::uint8_t kCrtlRxWarning = 0x04U;
constexpr std::uint8_t kCrtlRxPassive = 0x10U;

gim6010_driver::CanFrame makeErrorFrame(std::uint32_t id_flags)
{
  gim6010_driver::CanFrame frame;
  frame.error = true;
  frame.id = id_flags;
  frame.dlc = 8;
  return frame;
}
}  // namespace

TEST(CanErrorDecode, ClassifiesBusOffAndAck)
{
  const auto frame = makeErrorFrame(kErrBusoff | kErrAck);
  const auto decoded = gim6010_driver::decodeErrorFrame(frame);
  EXPECT_TRUE(decoded.bus_off);
  EXPECT_TRUE(decoded.ack_error);
  EXPECT_FALSE(decoded.error_warning);
  EXPECT_FALSE(decoded.error_passive);
}

TEST(CanErrorDecode, ClassifiesControllerWarning)
{
  auto frame = makeErrorFrame(kErrCrtl);
  frame.data[1] = kCrtlRxWarning;
  const auto decoded = gim6010_driver::decodeErrorFrame(frame);
  EXPECT_TRUE(decoded.error_warning);
  EXPECT_FALSE(decoded.error_passive);
  EXPECT_FALSE(decoded.bus_off);
}

TEST(CanErrorDecode, ClassifiesControllerPassive)
{
  auto frame = makeErrorFrame(kErrCrtl);
  frame.data[1] = kCrtlRxPassive;
  const auto decoded = gim6010_driver::decodeErrorFrame(frame);
  EXPECT_TRUE(decoded.error_passive);
  EXPECT_FALSE(decoded.error_warning);
}

TEST(CanErrorDecode, ExtractsErrorCountersWhenPresent)
{
  auto frame = makeErrorFrame(kErrCnt);
  frame.data[6] = 3;
  frame.data[7] = 5;
  const auto decoded = gim6010_driver::decodeErrorFrame(frame);
  ASSERT_TRUE(decoded.has_counters);
  EXPECT_EQ(decoded.tx_error_counter, 3U);
  EXPECT_EQ(decoded.rx_error_counter, 5U);
}

TEST(CanDiagnostics, StateEscalatesButNeverDowngradesWithinAStatus)
{
  gim6010_driver::CanErrorStatus status;
  EXPECT_EQ(status.state, gim6010_driver::CanBusState::kErrorActive);

  gim6010_driver::DecodedCanError warning;
  warning.error_warning = true;
  gim6010_driver::recordCanError(status, warning);
  EXPECT_EQ(status.state, gim6010_driver::CanBusState::kErrorWarning);
  EXPECT_EQ(status.warning_frames, 1U);

  gim6010_driver::DecodedCanError passive;
  passive.error_passive = true;
  gim6010_driver::recordCanError(status, passive);
  EXPECT_EQ(status.state, gim6010_driver::CanBusState::kErrorPassive);
  EXPECT_EQ(status.passive_frames, 1U);

  // A later warning-level frame is still counted, but must not pull the
  // sticky state back down from the worse passive condition already seen.
  gim6010_driver::recordCanError(status, warning);
  EXPECT_EQ(status.state, gim6010_driver::CanBusState::kErrorPassive);
  EXPECT_EQ(status.warning_frames, 2U);

  EXPECT_EQ(status.total_frames, 3U);
}

TEST(CanDiagnostics, BusOffIsTheMostSevereState)
{
  gim6010_driver::CanErrorStatus status;
  gim6010_driver::DecodedCanError bus_off;
  bus_off.bus_off = true;
  gim6010_driver::recordCanError(status, bus_off);
  EXPECT_EQ(status.state, gim6010_driver::CanBusState::kBusOff);
  EXPECT_EQ(status.bus_off_frames, 1U);
}

TEST(CanDiagnostics, RecordsErrorCounterSnapshot)
{
  gim6010_driver::CanErrorStatus status;
  gim6010_driver::DecodedCanError counters;
  counters.has_counters = true;
  counters.tx_error_counter = 10;
  counters.rx_error_counter = 20;
  gim6010_driver::recordCanError(status, counters);
  EXPECT_EQ(status.tx_error_counter, 10U);
  EXPECT_EQ(status.rx_error_counter, 20U);
}

TEST(CanDiagnostics, RecordCanRxDropIncrementsCounter)
{
  gim6010_driver::CanErrorStatus status;
  gim6010_driver::recordCanRxDrop(status);
  gim6010_driver::recordCanRxDrop(status);
  EXPECT_EQ(status.rx_dropped_frames, 2U);
}
