// SPDX-License-Identifier: Apache-2.0
#include "galata/hardware/serial_transport.hpp"
#include "galata/onboard/runtime.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace {

galata::hardware::InterfaceSpec specification() {
  galata::hardware::InterfaceSpec result;
  result.id = "runtime-replay-v1";
  result.sample_period_s = 0.01;
  result.sensor_channels = {{"airspeed_m_s", "m/s", "body"}};
  result.actuator_channels = {{"elevator_rad", "rad", "body"}};
  return result;
}

}  // namespace

TEST(OnboardRuntime, RunsOneForOneAndFaultsClosedOnInputTimeout) {
  using galata::hardware::Frame;
  galata::hardware::ReplayTransport transport({Frame{10, 2.0, {30.0}}, Frame{11, 2.01, {30.1}}});
  galata::hardware::ArmingInterlock interlock;
  galata::onboard::Runtime runtime(transport, interlock, {specification(), 0.1, 0.2});

  runtime.connect();
  EXPECT_EQ(runtime.state(), galata::onboard::RuntimeState::Ready);
  runtime.arm("operator-1", "ARM");
  EXPECT_EQ(runtime.state(), galata::onboard::RuntimeState::Armed);

  const auto controller = [](const Frame& sensor) {
    return Frame{sensor.sequence, sensor.timestamp_s, {sensor.values.front() * 0.001}};
  };
  EXPECT_TRUE(runtime.step(controller));
  EXPECT_TRUE(runtime.step(controller));
  EXPECT_EQ(runtime.completed_cycles(), 2U);
  EXPECT_EQ(runtime.metrics().observed_cycles, 2U);
  EXPECT_GT(runtime.metrics().controller_worst_case_s, 0.0);
  EXPECT_GT(runtime.metrics().cycle_worst_case_s, 0.0);
  ASSERT_EQ(transport.sent_frames().size(), 2U);
  EXPECT_DOUBLE_EQ(transport.sent_frames().back().values.front(), 0.0301);

  EXPECT_FALSE(runtime.step(controller));
  EXPECT_EQ(runtime.state(), galata::onboard::RuntimeState::Faulted);
  EXPECT_FALSE(interlock.armed());
  EXPECT_EQ(runtime.fault_reason(), "sensor receive timed out before a complete cycle");
}

TEST(OnboardRuntime, RunsOneCycleThroughThePosixSerialTransport) {
  const int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  ASSERT_GE(master, 0);
  ASSERT_EQ(::grantpt(master), 0);
  ASSERT_EQ(::unlockpt(master), 0);
  const char* slave_name = ::ptsname(master);
  ASSERT_NE(slave_name, nullptr);

  galata::hardware::InterfaceSpec serial_spec = specification();
  galata::hardware::SerialTransport transport({slave_name, 115200, 50});
  galata::hardware::ArmingInterlock interlock;
  galata::onboard::Runtime runtime(transport, interlock, {serial_spec, 0.1, 0.2});
  runtime.connect();
  runtime.arm("operator-serial", "ARM");

  const galata::hardware::Frame sensor{3, 0.0, {31.5}};
  const std::vector<std::uint8_t> sensor_packet =
      galata::hardware::FrameCodec::encode(sensor, sensor.values.size());
  ASSERT_EQ(::write(master, sensor_packet.data(), sensor_packet.size()),
            static_cast<ssize_t>(sensor_packet.size()));

  EXPECT_TRUE(runtime.step([](const galata::hardware::Frame& received) {
    return galata::hardware::Frame{
        received.sequence, received.timestamp_s, {received.values.front() * 0.001}};
  }));
  EXPECT_EQ(runtime.completed_cycles(), 1U);

  std::vector<std::uint8_t> actuator_packet(24U + sizeof(double) + 4U);
  std::size_t offset = 0;
  while (offset < actuator_packet.size()) {
    pollfd output{master, POLLIN, 0};
    ASSERT_GT(::poll(&output, 1, 100), 0);
    const ssize_t count =
        ::read(master, actuator_packet.data() + offset, actuator_packet.size() - offset);
    ASSERT_GT(count, 0);
    offset += static_cast<std::size_t>(count);
  }
  const auto actuator = galata::hardware::FrameCodec::decode(actuator_packet, 1);
  EXPECT_EQ(actuator.sequence, sensor.sequence);
  EXPECT_DOUBLE_EQ(actuator.timestamp_s, sensor.timestamp_s);
  EXPECT_DOUBLE_EQ(actuator.values.front(), 0.0315);
  EXPECT_EQ(runtime.state(), galata::onboard::RuntimeState::Armed);

  runtime.stop();
  EXPECT_FALSE(interlock.armed());
  ::close(master);
}

TEST(OnboardRuntime, RefusesMismatchedControllerOutputAndDisarmsBeforeThrowing) {
  using galata::hardware::Frame;
  galata::hardware::ReplayTransport transport({Frame{4, 1.0, {25.0}}});
  galata::hardware::ArmingInterlock interlock;
  galata::onboard::Runtime runtime(transport, interlock, {specification(), 0.1, 0.2});
  runtime.connect();
  runtime.arm("operator-1", "ARM");

  EXPECT_THROW((void)runtime.step([](const Frame& sensor) {
    return Frame{sensor.sequence + 1U, sensor.timestamp_s, {0.1}};
  }),
               std::invalid_argument);
  EXPECT_EQ(runtime.state(), galata::onboard::RuntimeState::Faulted);
  EXPECT_FALSE(interlock.armed());
  EXPECT_TRUE(transport.sent_frames().empty());
}

TEST(OnboardRuntime, DoesNotArmAfterTheTransportStopsBeingReady) {
  galata::hardware::ReplayTransport transport;
  galata::hardware::ArmingInterlock interlock;
  galata::onboard::Runtime runtime(transport, interlock, {specification(), 0.1, 0.2});
  runtime.connect();
  transport.disconnect();

  EXPECT_THROW(runtime.arm("operator-1", "ARM"), std::runtime_error);
  EXPECT_EQ(runtime.state(), galata::onboard::RuntimeState::Faulted);
  EXPECT_FALSE(interlock.armed());
}

TEST(OnboardRuntime, ControllerExceptionFaultsAndDisconnectsTheLink) {
  using galata::hardware::Frame;
  galata::hardware::ReplayTransport transport({Frame{0, 0.0, {25.0}}});
  galata::hardware::ArmingInterlock interlock;
  galata::onboard::Runtime runtime(transport, interlock, {specification(), 0.1, 0.2});
  runtime.connect();
  runtime.arm("operator-1", "ARM");

  EXPECT_THROW((void)runtime.step(
                   [](const Frame&) -> Frame { throw std::runtime_error("controller failed"); }),
               std::runtime_error);
  EXPECT_EQ(runtime.state(), galata::onboard::RuntimeState::Faulted);
  EXPECT_EQ(transport.state(), galata::hardware::LinkState::Disconnected);
  EXPECT_FALSE(interlock.armed());
}

TEST(OnboardRuntime, DetectsControllerExecutionBudgetOverrunAndDisarms) {
  using galata::hardware::Frame;
  galata::hardware::ReplayTransport transport({Frame{0, 0.0, {25.0}}});
  galata::hardware::ArmingInterlock interlock;
  galata::onboard::Runtime runtime(transport, interlock, {specification(), 0.001, 0.02});
  runtime.connect();
  runtime.arm("operator-1", "ARM");

  EXPECT_THROW((void)runtime.step([](const Frame& sensor) {
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    return Frame{sensor.sequence, sensor.timestamp_s, {0.1}};
  }),
               std::runtime_error);
  EXPECT_EQ(runtime.state(), galata::onboard::RuntimeState::Faulted);
  EXPECT_FALSE(interlock.armed());
  EXPECT_TRUE(transport.sent_frames().empty());
}

TEST(OnboardRuntime, RefusesAControllerBudgetLongerThanTheWatchdog) {
  using galata::hardware::Frame;
  galata::hardware::ReplayTransport transport({Frame{0, 0.0, {25.0}}});
  galata::hardware::ArmingInterlock interlock;
  EXPECT_THROW(
      {
        galata::onboard::Runtime runtime(transport, interlock, {specification(), 0.02, 0.01});
      },
      std::invalid_argument);
}
