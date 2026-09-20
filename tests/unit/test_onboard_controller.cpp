// SPDX-License-Identifier: Apache-2.0
#include "galata/hardware/interface.hpp"
#include "galata/onboard/controller_plugin.hpp"
#include "galata/onboard/runtime.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace {

galata::hardware::InterfaceSpec specification() {
  galata::hardware::InterfaceSpec result;
  result.id = "plugin-test-v1";
  result.sample_period_s = 0.01;
  result.sensor_channels = {{"airspeed_m_s", "m/s", "body"}};
  result.actuator_channels = {{"elevator_rad", "rad", "body"}};
  return result;
}

}  // namespace

TEST(OnboardControllerPlugin, RunsThroughTheSameGuardedRuntimeContract) {
#if defined(GALATA_TEST_CONTROLLER_PATH)
  const galata::hardware::InterfaceSpec interface = specification();
  const std::filesystem::path model_path =
      std::filesystem::temp_directory_path() / "galata-controller-plugin-model.bin";
  {
    std::ofstream model(model_path, std::ios::binary);
    ASSERT_TRUE(model);
    model << "model-artifact-v1\n";
  }
  galata::onboard::ControllerPlugin controller(
      GALATA_TEST_CONTROLLER_PATH, model_path, interface, "controlled-manifest-v1");
  galata::hardware::ReplayTransport transport({{7, 1.25, {31.5}}});
  galata::hardware::ArmingInterlock interlock;
  galata::onboard::Runtime runtime(transport, interlock, {interface, 0.1, 0.2});

  runtime.connect();
  runtime.arm("plugin-test", "ARM");
  ASSERT_TRUE(runtime.step(
      [&controller](const galata::hardware::Frame& sensor) { return controller.step(sensor); }));
  ASSERT_EQ(transport.sent_frames().size(), 1U);
  EXPECT_EQ(transport.sent_frames().front().sequence, 7U);
  EXPECT_DOUBLE_EQ(transport.sent_frames().front().timestamp_s, 1.25);
  EXPECT_DOUBLE_EQ(transport.sent_frames().front().values.front(), 0.0315);
  runtime.stop();
  std::error_code cleanup_error;
  std::filesystem::remove(model_path, cleanup_error);
#else
  GTEST_SKIP() << "test controller path was not configured";
#endif
}
