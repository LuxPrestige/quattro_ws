#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>

#include "gim6010_driver/gds68_protocol.hpp"

namespace
{
using gim6010_driver::Gds68Command;
constexpr double kTwoPi = 6.28318530717958647692;

std::uint32_t f32Bits(float value)
{
  std::uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}
}  // namespace

TEST(Gds68Protocol, ArbitrationIdRoundTrips)
{
  const auto id = gim6010_driver::makeArbitrationId(5, Gds68Command::kHeartbeat);
  EXPECT_EQ(id, (5U << 5) | 0x01U);
  const auto parsed = gim6010_driver::parseArbitrationId(id);
  EXPECT_EQ(parsed.first, 5U);
  EXPECT_EQ(parsed.second, Gds68Command::kHeartbeat);
}

TEST(Gds68Protocol, ArbitrationIdRejectsOutOfRangeNode)
{
  EXPECT_THROW(
    gim6010_driver::makeArbitrationId(
      gim6010_driver::kMaximumNodeId + 1U, Gds68Command::kHeartbeat),
    std::invalid_argument);
}

TEST(Gds68Protocol, ControllerModeEncodesBothFieldsLittleEndian)
{
  const auto payload = gim6010_driver::encodeControllerMode(
    gim6010_driver::ControlMode::kPosition, gim6010_driver::InputMode::kMit);
  EXPECT_EQ(payload[0], 3U);
  EXPECT_EQ(payload[1], 0U);
  EXPECT_EQ(payload[2], 0U);
  EXPECT_EQ(payload[3], 0U);
  EXPECT_EQ(payload[4], 9U);
  EXPECT_EQ(payload[5], 0U);
}

TEST(Gds68Protocol, DirectPositionEncodesFloatPositionLittleEndian)
{
  const auto payload = gim6010_driver::encodeDirectPosition(2.5F, 0.0F, 0.0F);
  const auto expected_bits = f32Bits(2.5F);
  EXPECT_EQ(payload[0], static_cast<std::uint8_t>(expected_bits & 0xFFU));
  EXPECT_EQ(payload[1], static_cast<std::uint8_t>((expected_bits >> 8) & 0xFFU));
  EXPECT_EQ(payload[2], static_cast<std::uint8_t>((expected_bits >> 16) & 0xFFU));
  EXPECT_EQ(payload[3], static_cast<std::uint8_t>((expected_bits >> 24) & 0xFFU));
}

TEST(Gds68Protocol, EncoderEstimatesRoundTripsThroughFloatBytes)
{
  std::array<std::uint8_t, 8> raw{};
  const auto position_bits = f32Bits(-1.25F);
  const auto velocity_bits = f32Bits(3.0F);
  std::memcpy(raw.data(), &position_bits, 4);
  std::memcpy(raw.data() + 4, &velocity_bits, 4);
  const auto decoded = gim6010_driver::decodeEncoderEstimates(raw.data(), raw.size());
  EXPECT_FLOAT_EQ(decoded.first, -1.25F);
  EXPECT_FLOAT_EQ(decoded.second, 3.0F);
}

TEST(Gds68Protocol, EncoderEstimatesRejectsShortPayload)
{
  std::array<std::uint8_t, 4> raw{};
  EXPECT_THROW(gim6010_driver::decodeEncoderEstimates(raw.data(), raw.size()), std::length_error);
}

TEST(Gds68Protocol, GetErrorRequestCarriesCategoryInFirstByte)
{
  // Manual section 4.1.2 Error_Type: 0=motor, 1=encoder, 3=controller,
  // 4=system -- index 2 is not assigned.
  const auto payload = gim6010_driver::encodeGetErrorRequest(
    gim6010_driver::ErrorType::kController);
  EXPECT_EQ(payload[0], 3U);
}

TEST(Gds68Protocol, GetErrorResponseDecodes32BitCategoryFromByteZero)
{
  // kEncoder/kController/kSystem: uint32 across bytes 0-3, no type tag in
  // the payload (the response reuses the same 0x03 arbitration id for every
  // category, so the caller must already know which one it asked for).
  std::array<std::uint8_t, 4> raw{0x78U, 0x56U, 0x34U, 0x12U};
  const auto decoded = gim6010_driver::decodeGetErrorResponse(
    gim6010_driver::ErrorType::kEncoder, raw.data(), raw.size());
  EXPECT_EQ(decoded, 0x12345678U);
}

TEST(Gds68Protocol, GetErrorResponseDecodes64BitMotorErrorFromAllEightBytes)
{
  std::array<std::uint8_t, 8> raw{
    0x01U, 0x00U, 0x00U, 0x00U, 0x02U, 0x00U, 0x00U, 0x00U};
  const auto decoded = gim6010_driver::decodeGetErrorResponse(
    gim6010_driver::ErrorType::kMotor, raw.data(), raw.size());
  EXPECT_EQ(decoded, 0x0000000200000001ULL);
}

TEST(Gds68Protocol, GetErrorResponseRejectsShortPayload)
{
  std::array<std::uint8_t, 3> raw{};
  EXPECT_THROW(
    gim6010_driver::decodeGetErrorResponse(gim6010_driver::ErrorType::kSystem, raw.data(),
    raw.size()),
    std::length_error);
  std::array<std::uint8_t, 7> short_motor_raw{};
  EXPECT_THROW(
    gim6010_driver::decodeGetErrorResponse(
      gim6010_driver::ErrorType::kMotor, short_motor_raw.data(), short_motor_raw.size()),
    std::length_error);
}

TEST(Gds68Protocol, EncoderCountDecodesShadowAndCprCounts)
{
  std::array<std::uint8_t, 8> raw{};
  const std::uint32_t shadow = 0xFFFFFFFEU;  // -2 as two's complement
  const std::uint32_t count_in_cpr = 100U;
  std::memcpy(raw.data(), &shadow, 4);
  std::memcpy(raw.data() + 4, &count_in_cpr, 4);
  const auto decoded = gim6010_driver::decodeEncoderCount(raw.data(), raw.size());
  EXPECT_EQ(decoded.shadow_count, -2);
  EXPECT_EQ(decoded.count_in_cpr, 100);
}

TEST(Gds68Protocol, HeartbeatDecodesEachFieldFromItsDocumentedByte)
{
  std::array<std::uint8_t, 8> raw{};
  const std::uint32_t axis_error = 0x000000A5U;
  std::memcpy(raw.data(), &axis_error, 4);
  raw[4] = 8U;    // axis_state: closed-loop control
  raw[5] = 0x05U; // flags
  raw[6] = 0xFFU; // reserved
  raw[7] = 42U;   // life
  const auto heartbeat = gim6010_driver::decodeHeartbeat(raw.data(), raw.size());
  EXPECT_EQ(heartbeat.axis_error, axis_error);
  EXPECT_EQ(heartbeat.axis_state, 8U);
  EXPECT_EQ(heartbeat.flags, 0x05U);
  EXPECT_EQ(heartbeat.life, 42U);
}

TEST(Gds68Protocol, RotorToOutputConversionMatchesGearRatio)
{
  // GIM6010-8's 8:1 gearbox: 8 rotor revolutions = 1 output revolution =
  // 2*pi output radians.
  EXPECT_DOUBLE_EQ(gim6010_driver::rotorRevToOutputRad(8.0, 8.0), kTwoPi);
  EXPECT_DOUBLE_EQ(gim6010_driver::rotorRevPerSecToOutputRadPerSec(8.0, 8.0), kTwoPi);
  // One full rotor turn only advances the output shaft by 1/8 turn.
  EXPECT_DOUBLE_EQ(gim6010_driver::rotorRevToOutputRad(1.0, 8.0), kTwoPi / 8.0);
  // A direct 1:1 coupling is the identity conversion.
  EXPECT_DOUBLE_EQ(gim6010_driver::rotorRevToOutputRad(1.0, 1.0), kTwoPi);
}

TEST(Gds68Protocol, LimitsAndGainsEncodeAsLittleEndianFloatPairs)
{
  const auto limits = gim6010_driver::encodeLimits(5.0F, 8.0F);
  const std::uint32_t decoded_bits =
    static_cast<std::uint32_t>(limits[0]) | (static_cast<std::uint32_t>(limits[1]) << 8) |
    (static_cast<std::uint32_t>(limits[2]) << 16) | (static_cast<std::uint32_t>(limits[3]) << 24);
  EXPECT_EQ(decoded_bits, f32Bits(5.0F));

  const auto gains = gim6010_driver::encodeVelocityGains(0.16F, 0.32F);
  float decoded_gain{};
  std::memcpy(&decoded_gain, gains.data(), 4);
  EXPECT_FLOAT_EQ(decoded_gain, 0.16F);
}
