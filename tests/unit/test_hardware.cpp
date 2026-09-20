// SPDX-License-Identifier: Apache-2.0
#include "galata/core/sha256.hpp"
#include "galata/hardware/can_transport.hpp"
#include "galata/hardware/interface.hpp"
#include "galata/hardware/serial_transport.hpp"
#include "galata/hardware/udp_transport.hpp"
#include "galata/onboard/deployment.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

namespace {

galata::hardware::InterfaceSpec specification() {
  galata::hardware::InterfaceSpec result;
  result.id = "replay-bench-v1";
  result.sample_period_s = 0.01;
  result.sensor_channels = {{"airspeed_m_s", "m/s", "body"}, {"altitude_m", "m", "ned"}};
  result.actuator_channels = {{"elevator_rad", "rad", "body"}};
  return result;
}

galata::hardware::TransportProfile transport_profile() {
  return {"replay-profile-v1", "replay", "reviewed-sil-replay", 20, 20, 0.02, true};
}

}  // namespace

TEST(HardwareInterface, UdpTransportRoundTripsCodecFramesAndTimesOutBoundedly) {
  const int server = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(server, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons(0);
  ASSERT_EQ(::bind(server, reinterpret_cast<const sockaddr*>(&address), sizeof(address)), 0);
  socklen_t address_length = sizeof(address);
  ASSERT_EQ(::getsockname(server, reinterpret_cast<sockaddr*>(&address), &address_length), 0);

  galata::hardware::InterfaceSpec udp_spec = specification();
  udp_spec.sensor_channels = {{"airspeed_m_s", "m/s", "body"}};
  udp_spec.actuator_channels = {{"elevator_rad", "rad", "body"}};
  galata::hardware::UdpTransport transport({"127.0.0.1", ntohs(address.sin_port), 20, 0});
  transport.connect(udp_spec);

  transport.send({0, 0.0, {0.15}});
  std::vector<std::uint8_t> first_output(24U + sizeof(double) + 4U);
  sockaddr_in peer{};
  socklen_t peer_length = sizeof(peer);
  ASSERT_EQ(::recvfrom(server,
                       first_output.data(),
                       first_output.size(),
                       0,
                       reinterpret_cast<sockaddr*>(&peer),
                       &peer_length),
            static_cast<ssize_t>(first_output.size()));

  const galata::hardware::Frame sensor{7, 2.0, {31.5}};
  const std::vector<std::uint8_t> sensor_packet =
      galata::hardware::FrameCodec::encode(sensor, sensor.values.size());
  ASSERT_EQ(::sendto(server,
                     sensor_packet.data(),
                     sensor_packet.size(),
                     0,
                     reinterpret_cast<const sockaddr*>(&peer),
                     peer_length),
            static_cast<ssize_t>(sensor_packet.size()));

  galata::hardware::Frame received;
  ASSERT_TRUE(transport.receive(received));
  EXPECT_EQ(received.sequence, sensor.sequence);
  EXPECT_DOUBLE_EQ(received.values.front(), sensor.values.front());

  transport.send({1, 0.01, {0.15}});
  std::vector<std::uint8_t> output_packet(24U + sizeof(double) + 4U);
  ASSERT_EQ(::recv(server, output_packet.data(), output_packet.size(), 0),
            static_cast<ssize_t>(output_packet.size()));
  const auto output = galata::hardware::FrameCodec::decode(output_packet, 1);
  EXPECT_EQ(output.sequence, 1u);
  EXPECT_DOUBLE_EQ(output.values.front(), 0.15);

  EXPECT_FALSE(transport.receive(received));
  transport.disconnect();
  ::close(server);
}

TEST(HardwareInterface, SerialTransportRoundTripsThroughAPseudoTerminal) {
  const int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  ASSERT_GE(master, 0);
  ASSERT_EQ(::grantpt(master), 0);
  ASSERT_EQ(::unlockpt(master), 0);
  const char* slave_name = ::ptsname(master);
  ASSERT_NE(slave_name, nullptr);

  galata::hardware::InterfaceSpec serial_spec = specification();
  serial_spec.sensor_channels = {{"airspeed_m_s", "m/s", "body"}};
  serial_spec.actuator_channels = {{"elevator_rad", "rad", "body"}};
  galata::hardware::SerialTransport transport({slave_name, 115200, 50});
  transport.connect(serial_spec);

  const galata::hardware::Frame sensor{7, 2.0, {31.5}};
  const std::vector<std::uint8_t> sensor_packet =
      galata::hardware::FrameCodec::encode(sensor, sensor.values.size());
  ASSERT_EQ(::write(master, sensor_packet.data(), sensor_packet.size()),
            static_cast<ssize_t>(sensor_packet.size()));
  galata::hardware::Frame received;
  ASSERT_TRUE(transport.receive(received));
  EXPECT_EQ(received.sequence, sensor.sequence);
  EXPECT_DOUBLE_EQ(received.values.front(), sensor.values.front());

  transport.send({0, 0.0, {0.15}});
  std::vector<std::uint8_t> output_packet(24U + sizeof(double) + 4U);
  std::size_t offset = 0;
  while (offset < output_packet.size()) {
    pollfd output{master, POLLIN, 0};
    ASSERT_GT(::poll(&output, 1, 100), 0);
    const ssize_t count =
        ::read(master, output_packet.data() + offset, output_packet.size() - offset);
    ASSERT_GT(count, 0);
    offset += static_cast<std::size_t>(count);
  }
  const auto decoded = galata::hardware::FrameCodec::decode(output_packet, 1);
  EXPECT_EQ(decoded.sequence, 0u);
  EXPECT_DOUBLE_EQ(decoded.values.front(), 0.15);

  EXPECT_FALSE(transport.receive(received));
  transport.disconnect();
  ::close(master);
}

TEST(HardwareInterface, UdpTransportFixedLocalPortReceivesBeforeFirstActuatorFrame) {
  const int server = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(server, 0);
  sockaddr_in server_address{};
  server_address.sin_family = AF_INET;
  server_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  server_address.sin_port = htons(0);
  ASSERT_EQ(::bind(server,
                   reinterpret_cast<const sockaddr*>(&server_address),
                   sizeof(server_address)),
            0);
  socklen_t address_length = sizeof(server_address);
  ASSERT_EQ(::getsockname(server,
                          reinterpret_cast<sockaddr*>(&server_address),
                          &address_length),
            0);

  const int local_port_probe = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(local_port_probe, 0);
  sockaddr_in local_address{};
  local_address.sin_family = AF_INET;
  local_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  local_address.sin_port = htons(0);
  ASSERT_EQ(::bind(local_port_probe,
                   reinterpret_cast<const sockaddr*>(&local_address),
                   sizeof(local_address)),
            0);
  address_length = sizeof(local_address);
  ASSERT_EQ(::getsockname(local_port_probe,
                          reinterpret_cast<sockaddr*>(&local_address),
                          &address_length),
            0);
  const auto local_port = ntohs(local_address.sin_port);
  ::close(local_port_probe);

  galata::hardware::InterfaceSpec udp_spec = specification();
  udp_spec.sensor_channels = {{"airspeed_m_s", "m/s", "body"}};
  udp_spec.actuator_channels = {{"elevator_rad", "rad", "body"}};
  galata::hardware::UdpTransport transport(
      {"127.0.0.1", ntohs(server_address.sin_port), 20, local_port});
  transport.connect(udp_spec);

  const galata::hardware::Frame sensor{7, 0.0, {31.5}};
  const std::vector<std::uint8_t> sensor_packet =
      galata::hardware::FrameCodec::encode(sensor, sensor.values.size());
  sockaddr_in runner_address{};
  runner_address.sin_family = AF_INET;
  runner_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  runner_address.sin_port = htons(local_port);
  ASSERT_EQ(::sendto(server,
                     sensor_packet.data(),
                     sensor_packet.size(),
                     0,
                     reinterpret_cast<const sockaddr*>(&runner_address),
                     sizeof(runner_address)),
            static_cast<ssize_t>(sensor_packet.size()));

  galata::hardware::Frame received;
  ASSERT_TRUE(transport.receive(received));
  EXPECT_EQ(received.sequence, sensor.sequence);
  EXPECT_DOUBLE_EQ(received.values.front(), sensor.values.front());

  transport.send({0, 0.0, {0.0315}});
  std::vector<std::uint8_t> output_packet(24U + sizeof(double) + 4U);
  sockaddr_in peer{};
  socklen_t peer_length = sizeof(peer);
  ASSERT_EQ(::recvfrom(server,
                       output_packet.data(),
                       output_packet.size(),
                       0,
                       reinterpret_cast<sockaddr*>(&peer),
                       &peer_length),
            static_cast<ssize_t>(output_packet.size()));
  const auto output = galata::hardware::FrameCodec::decode(output_packet, 1);
  EXPECT_EQ(output.sequence, 0u);
  EXPECT_DOUBLE_EQ(output.values.front(), 0.0315);

  transport.disconnect();
  ::close(server);
}

TEST(HardwareInterface, CanFdEndpointRequiresAConcreteBoundedSocketCanContract) {
  EXPECT_THROW(galata::hardware::CanFdTransport({}), std::invalid_argument);
  EXPECT_THROW(galata::hardware::CanFdTransport({"can0", 0, 0x121, 100, 100}),
               std::invalid_argument);
  EXPECT_THROW(galata::hardware::CanFdTransport({"can0", 0x120, 0x120, 100, 100}),
               std::invalid_argument);
  EXPECT_THROW(galata::hardware::CanFdTransport({"can0", 0x20000000, 0x121, 100, 100}),
               std::invalid_argument);
  EXPECT_NO_THROW(galata::hardware::CanFdTransport({"can0", 0x120, 0x121, 100, 100}));

  galata::hardware::InterfaceSpec oversized = specification();
  oversized.sensor_channels = {
      {"sensor_0", "m/s", "body"}, {"sensor_1", "m/s", "body"},
      {"sensor_2", "m/s", "body"}, {"sensor_3", "m/s", "body"},
      {"sensor_4", "m/s", "body"}, {"sensor_5", "m/s", "body"},
  };
  galata::hardware::CanFdTransport width_checked(
      {"__galata_missing_can__", 0x120, 0x121, 100, 100});
  EXPECT_THROW(width_checked.connect(oversized), std::invalid_argument);
  EXPECT_EQ(width_checked.state(), galata::hardware::LinkState::Disconnected);

  galata::hardware::CanFdTransport unavailable(
      {"__galata_missing_can__", 0x120, 0x121, 100, 100});
  EXPECT_THROW(unavailable.connect(specification()), std::runtime_error);
  EXPECT_EQ(unavailable.state(), galata::hardware::LinkState::Faulted);
}

TEST(HardwareInterface, TransportProfileRequiresBoundedLinkAndIndependentStop) {
  EXPECT_NO_THROW(galata::hardware::validate_transport_profile(transport_profile(), 0.01));

  auto invalid = transport_profile();
  invalid.transport = "vendor-magical-bus";
  EXPECT_THROW(galata::hardware::validate_transport_profile(invalid, 0.01), std::invalid_argument);

  invalid = transport_profile();
  invalid.watchdog_timeout_s = 0.005;
  EXPECT_THROW(galata::hardware::validate_transport_profile(invalid, 0.01), std::invalid_argument);

  invalid = transport_profile();
  invalid.emergency_stop_required = false;
  EXPECT_THROW(galata::hardware::validate_transport_profile(invalid, 0.01), std::invalid_argument);

  auto udp = transport_profile();
  udp.transport = "udp";
  udp.endpoint = "127.0.0.1:9000";
  EXPECT_THROW(galata::hardware::validate_transport_profile(udp, 0.01), std::invalid_argument);
  udp.endpoint = "127.0.0.1:9000|9001";
  EXPECT_NO_THROW(galata::hardware::validate_transport_profile(udp, 0.01));
}

TEST(HardwareInterface, SerialAndUdpEndpointsRequireIndependentTransmitDeadlines) {
  EXPECT_THROW(galata::hardware::SerialTransport({"/dev/null", 115200, 20, 0}),
               std::invalid_argument);
  EXPECT_THROW(galata::hardware::UdpTransport({"127.0.0.1", 9000, 20, 0, 0}),
               std::invalid_argument);
  EXPECT_NO_THROW(galata::hardware::UdpTransport({"127.0.0.1", 9000, 20, 0, 20}));
}

TEST(HardwareInterface, UdpTransportFaultsOnMalformedDatagram) {
  const int server = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(server, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons(0);
  ASSERT_EQ(::bind(server, reinterpret_cast<const sockaddr*>(&address), sizeof(address)), 0);
  socklen_t address_length = sizeof(address);
  ASSERT_EQ(::getsockname(server, reinterpret_cast<sockaddr*>(&address), &address_length), 0);

  galata::hardware::InterfaceSpec udp_spec = specification();
  udp_spec.sensor_channels = {{"airspeed_m_s", "m/s", "body"}};
  udp_spec.actuator_channels = {{"elevator_rad", "rad", "body"}};
  galata::hardware::UdpTransport transport({"127.0.0.1", ntohs(address.sin_port), 20, 0});
  galata::hardware::ArmingInterlock interlock;
  galata::hardware::GuardedTransport guarded(transport, interlock);
  guarded.connect(udp_spec);
  interlock.arm("operator-1", "ARM");

  guarded.send({0, 0.0, {0.15}});
  std::vector<std::uint8_t> first_output(24U + sizeof(double) + 4U);
  sockaddr_in peer{};
  socklen_t peer_length = sizeof(peer);
  ASSERT_EQ(::recvfrom(server,
                       first_output.data(),
                       first_output.size(),
                       0,
                       reinterpret_cast<sockaddr*>(&peer),
                       &peer_length),
            static_cast<ssize_t>(first_output.size()));

  const std::vector<std::uint8_t> malformed{0x00U, 0x01U, 0x02U};
  ASSERT_EQ(::sendto(server,
                     malformed.data(),
                     malformed.size(),
                     0,
                     reinterpret_cast<const sockaddr*>(&peer),
                     peer_length),
            static_cast<ssize_t>(malformed.size()));
  galata::hardware::Frame received;
  EXPECT_THROW((void)guarded.receive(received), std::invalid_argument);
  EXPECT_EQ(transport.state(), galata::hardware::LinkState::Faulted);
  EXPECT_FALSE(interlock.armed());
  EXPECT_THROW(guarded.send({1, 0.01, {0.15}}), std::logic_error);
  ::close(server);
}

TEST(HardwareInterface, GuardedTransportDisarmsWhenTheUnderlyingLinkIsLost) {
  galata::hardware::ReplayTransport replay;
  galata::hardware::ArmingInterlock interlock;
  galata::hardware::GuardedTransport guarded(replay, interlock);
  guarded.connect(specification());
  interlock.arm("operator-1", "ARM");
  EXPECT_TRUE(interlock.armed());

  replay.disconnect();
  EXPECT_EQ(guarded.state(), galata::hardware::LinkState::Disconnected);
  EXPECT_FALSE(interlock.armed());
  EXPECT_THROW(guarded.send({0, 0.0, {0.02}}), std::logic_error);
}

TEST(HardwareInterface, ReplayIsDeterministicAndNeverArmsItself) {
  using galata::hardware::Frame;
  using galata::hardware::ReplayTransport;

  ReplayTransport transport({Frame{0, 0.0, {25.0, 100.0}}, Frame{1, 0.01, {25.1, 100.1}}});
  transport.connect(specification());

  Frame received;
  ASSERT_TRUE(transport.receive(received));
  EXPECT_EQ(received.sequence, 0u);
  EXPECT_TRUE(transport.receive(received));
  EXPECT_FALSE(transport.receive(received));
  EXPECT_EQ(transport.state(), galata::hardware::LinkState::Ready);
  EXPECT_TRUE(transport.sent_frames().empty());

  transport.send(Frame{0, 0.0, {0.02}});
  ASSERT_EQ(transport.sent_frames().size(), 1u);
  EXPECT_DOUBLE_EQ(transport.sent_frames().front().values.front(), 0.02);
}

TEST(HardwareInterface, FrameCodecRoundTripsAndRejectsCorruption) {
  const galata::hardware::Frame original{42, 1.25, {25.0, -0.5, 3.75}};
  const std::vector<std::uint8_t> packet =
      galata::hardware::FrameCodec::encode(original, original.values.size());
  ASSERT_EQ(packet.size(), 24U + 3U * sizeof(double) + 4U);
  EXPECT_EQ(packet[0], static_cast<std::uint8_t>('G'));
  EXPECT_EQ(packet[4], galata::hardware::FrameCodec::kVersion);

  const galata::hardware::Frame decoded =
      galata::hardware::FrameCodec::decode(packet, original.values.size());
  EXPECT_EQ(decoded.sequence, original.sequence);
  EXPECT_DOUBLE_EQ(decoded.timestamp_s, original.timestamp_s);
  EXPECT_EQ(decoded.values, original.values);

  std::vector<std::uint8_t> corrupted = packet;
  corrupted[24] ^= 0x01U;
  EXPECT_THROW((void)galata::hardware::FrameCodec::decode(corrupted, original.values.size()),
               std::invalid_argument);
  corrupted = packet;
  corrupted.pop_back();
  EXPECT_THROW((void)galata::hardware::FrameCodec::decode(corrupted, original.values.size()),
               std::invalid_argument);
}

TEST(HardwareInterface, InvalidFramesAndOrderingAreRefused) {
  using galata::hardware::Frame;
  using galata::hardware::ReplayTransport;

  EXPECT_THROW(
      ReplayTransport({Frame{1, 0.0, {25.0}}, Frame{1, 0.01, {25.1}}}).connect(specification()),
      std::invalid_argument);

  ReplayTransport transport;
  transport.connect(specification());
  EXPECT_THROW(transport.send(Frame{0, 0.0, {0.01, 0.02}}), std::invalid_argument);
  transport.send(Frame{1, 0.0, {0.01}});
  EXPECT_THROW(transport.send(Frame{1, 0.01, {0.02}}), std::invalid_argument);
}

TEST(HardwareInterface, ArmingRequiresAnExplicitOperatorConfirmation) {
  galata::hardware::ArmingInterlock interlock;
  EXPECT_FALSE(interlock.armed());
  EXPECT_THROW(interlock.require_armed(), std::logic_error);
  EXPECT_THROW(interlock.arm("operator-1", "yes"), std::invalid_argument);
  interlock.arm("operator-1", "ARM");
  EXPECT_TRUE(interlock.armed());
  EXPECT_EQ(interlock.operator_id(), "operator-1");
  interlock.disarm();
  EXPECT_FALSE(interlock.armed());
}

TEST(HardwareInterface, GuardedTransportBlocksOutputUntilArmedAndDisarmsOnDisconnect) {
  galata::hardware::ReplayTransport replay;
  galata::hardware::ArmingInterlock interlock;
  galata::hardware::GuardedTransport guarded(replay, interlock);
  guarded.connect(specification());

  EXPECT_THROW(guarded.send({0, 0.0, {0.02}}), std::logic_error);
  interlock.arm("operator-1", "ARM");
  guarded.send({0, 0.0, {0.02}});
  ASSERT_EQ(replay.sent_frames().size(), 1U);

  guarded.disconnect();
  EXPECT_FALSE(interlock.armed());
  EXPECT_EQ(guarded.state(), galata::hardware::LinkState::Disconnected);
}

TEST(OnboardDeployment, ManifestIsDeterministicAndExplicitlyNotQualified) {
  galata::onboard::DeploymentSpec deployment;
  deployment.target_platform = "example-flight-computer";
  deployment.target_identity = {"airframe-01", "fcu-example-v1", "firmware-build-001",
                                "estop-chain-01"};
  deployment.model_description = "vehicle model sha";
  deployment.controller_description = "controller sha";
  deployment.failsafe_action = "hold last safe command and disarm";
  deployment.max_controller_time_s = 0.005;
  deployment.interface = specification();
  deployment.transport_profile = transport_profile();
  deployment.artifacts = {{"model", std::string(64, 'a')}, {"controller", std::string(64, 'b')}};

  const auto first = galata::onboard::build_manifest_package(deployment);
  const auto second = galata::onboard::build_manifest_package(deployment);
  EXPECT_EQ(first.manifest, second.manifest);
  EXPECT_EQ(first.manifest_sha256, second.manifest_sha256);
  EXPECT_DOUBLE_EQ(first.max_controller_time_s, 0.005);
  EXPECT_FALSE(first.contains_executable);
  EXPECT_EQ(first.qualification_state, "not_qualified");
  EXPECT_NE(first.manifest.find("external_arming_required=true"), std::string::npos);
  EXPECT_NE(first.manifest.find("target.flight_computer_id=fcu-example-v1"), std::string::npos);
  EXPECT_NE(first.manifest.find("hardware.transport=replay"), std::string::npos);
  EXPECT_EQ(first.transport_profile.id, "replay-profile-v1");
  EXPECT_NO_THROW(galata::onboard::verify_manifest_package(first));
  const auto parsed = galata::onboard::parse_manifest_package(first.manifest);
  EXPECT_DOUBLE_EQ(parsed.max_controller_time_s, 0.005);
  EXPECT_EQ(parsed.transport_profile.endpoint, "reviewed-sil-replay");

  deployment.failsafe_action = "disarm\non-link-loss";
  EXPECT_THROW((void)galata::onboard::build_manifest_package(deployment), std::invalid_argument);

  deployment.failsafe_action = "hold last safe command and disarm";
  deployment.max_controller_time_s = 0.0;
  EXPECT_THROW((void)galata::onboard::build_manifest_package(deployment), std::invalid_argument);

  auto tampered = first;
  tampered.manifest += "tampered=true\n";
  EXPECT_THROW(galata::onboard::verify_manifest_package(tampered), std::invalid_argument);

  auto role_tampered = first;
  role_tampered.artifacts.front().role = "telemetry";
  EXPECT_THROW(galata::onboard::verify_manifest_package(role_tampered), std::invalid_argument);

  auto budget_tampered = first;
  budget_tampered.max_controller_time_s = 0.010;
  EXPECT_THROW(galata::onboard::verify_manifest_package(budget_tampered), std::invalid_argument);

  auto target_tampered = first;
  target_tampered.target_identity.firmware_id = "different-firmware-build";
  EXPECT_THROW(galata::onboard::verify_manifest_package(target_tampered), std::invalid_argument);

  auto profile_tampered = first;
  profile_tampered.transport_profile.watchdog_timeout_s = 0.03;
  EXPECT_THROW(galata::onboard::verify_manifest_package(profile_tampered), std::invalid_argument);

  auto undeclared = first;
  undeclared.manifest +=
      "artifact.99.role=telemetry\n"
      "artifact.99.sha256="
      + std::string(64, 'c') + "\n";
  undeclared.manifest_sha256 = galata::core::sha256(undeclared.manifest);
  EXPECT_THROW(galata::onboard::verify_manifest_package(undeclared), std::invalid_argument);
}

TEST(OnboardDeployment, StagesOnlyVerifiedArtifactsIntoANewAtomicDirectory) {
  const std::filesystem::path root =
      std::filesystem::temp_directory_path() / "galata-onboard-stage-test";
  std::error_code cleanup_error;
  std::filesystem::remove_all(root, cleanup_error);
  std::filesystem::create_directories(root / "sources");

  const std::string model_bytes = "model-artifact-v1\n";
  const std::string controller_bytes = "controller-artifact-v1\n";
  const std::filesystem::path model_path = root / "sources" / "model.bin";
  const std::filesystem::path controller_path = root / "sources" / "controller.bin";
  {
    std::ofstream model(model_path, std::ios::binary);
    ASSERT_TRUE(model);
    model << model_bytes;
    std::ofstream controller(controller_path, std::ios::binary);
    ASSERT_TRUE(controller);
    controller << controller_bytes;
  }

  galata::onboard::DeploymentSpec deployment;
  deployment.target_platform = "example-flight-computer";
  deployment.target_identity = {"airframe-01", "fcu-example-v1", "firmware-build-001",
                                "estop-chain-01"};
  deployment.model_description = "vehicle model sha";
  deployment.controller_description = "controller sha";
  deployment.failsafe_action = "hold last safe command and disarm";
  deployment.max_controller_time_s = 0.005;
  deployment.interface = specification();
  deployment.transport_profile = transport_profile();
  deployment.artifacts = {{"model", galata::core::sha256(model_bytes)},
                          {"controller", galata::core::sha256(controller_bytes)}};
  const auto package = galata::onboard::build_manifest_package(deployment);
  const std::filesystem::path destination = root / "staged-package";

  const auto receipt = galata::onboard::stage_manifest_package(
      package, {{"model", model_path}, {"controller", controller_path}}, destination);
  EXPECT_EQ(receipt.destination, destination);
  EXPECT_EQ(receipt.manifest_sha256, package.manifest_sha256);
  EXPECT_FALSE(receipt.contains_executable);
  EXPECT_EQ(receipt.qualification_state, "not_qualified");
  ASSERT_TRUE(std::filesystem::is_directory(destination));
  EXPECT_EQ(galata::core::sha256([&]() {
              std::ifstream input(destination / "onboard.manifest", std::ios::binary);
              return std::string(std::istreambuf_iterator<char>(input),
                                 std::istreambuf_iterator<char>());
            }()),
            package.manifest_sha256);
  EXPECT_EQ(galata::core::sha256([&]() {
              std::ifstream input(destination / "artifacts" / "model", std::ios::binary);
              return std::string(std::istreambuf_iterator<char>(input),
                                 std::istreambuf_iterator<char>());
            }()),
            deployment.artifacts[0].sha256);
  EXPECT_THROW((void)galata::onboard::stage_manifest_package(
                   package, {{"model", model_path}, {"controller", controller_path}}, destination),
               std::invalid_argument);

  std::filesystem::remove_all(root, cleanup_error);
}
