// SPDX-License-Identifier: Apache-2.0
// Study-facing construction of the target-neutral onboard handoff manifest.

#include "galata/pipeline/artifacts.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace galata::pipeline {
namespace {

std::vector<hardware::ChannelSpec> channels_from(const ValuePtr& parent, const std::string& key) {
  const ValuePtr value = parent->get(key);
  if (!value) {
    throw std::invalid_argument("onboard.manifest: interface." + key + " is required");
  }
  std::vector<hardware::ChannelSpec> channels;
  for (const ValuePtr& entry : value->as_list()) {
    if (!entry || entry->kind() != Value::Kind::Map) {
      throw std::invalid_argument("onboard.manifest: interface." + key + " entries must be maps");
    }
    channels.push_back(
        {entry->string_at("name"), entry->string_at("unit"), entry->string_at("frame")});
  }
  return channels;
}

hardware::InterfaceSpec interface_from(const ValuePtr& value) {
  if (!value || value->kind() != Value::Kind::Map) {
    throw std::invalid_argument("onboard.manifest: interface must be a map");
  }
  hardware::InterfaceSpec result;
  result.id = value->string_at("id");
  result.sample_period_s = value->number_at("sample_period_s");
  result.external_arming_required = value->bool_at("external_arming_required", true);
  result.sensor_channels = channels_from(value, "sensor_channels");
  result.actuator_channels = channels_from(value, "actuator_channels");
  return result;
}

hardware::TargetIdentity target_identity_from(const ValuePtr& value) {
  if (!value || value->kind() != Value::Kind::Map) {
    throw std::invalid_argument("onboard.manifest: target_identity must be a map");
  }
  return {value->string_at("hardware_id"),
          value->string_at("flight_computer_id"),
          value->string_at("firmware_id"),
          value->string_at("emergency_stop_id")};
}

std::uint32_t positive_uint32(const ValuePtr& value, const std::string& key) {
  const double number = value->number_at(key);
  if (!(number > 0.0) || !std::isfinite(number) || std::floor(number) != number
      || number > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
    throw std::invalid_argument("onboard.manifest: " + key + " must be a positive uint32");
  }
  return static_cast<std::uint32_t>(number);
}

hardware::TransportProfile transport_profile_from(const ValuePtr& value) {
  if (!value || value->kind() != Value::Kind::Map) {
    throw std::invalid_argument("onboard.manifest: transport_profile must be a map");
  }
  hardware::TransportProfile result;
  result.id = value->string_at("id");
  result.transport = value->string_at("transport");
  result.endpoint = value->string_at("endpoint");
  result.receive_timeout_ms = positive_uint32(value, "receive_timeout_ms");
  result.transmit_timeout_ms = positive_uint32(value, "transmit_timeout_ms");
  result.watchdog_timeout_s = value->number_at("watchdog_timeout_s");
  result.emergency_stop_required = value->bool_at("emergency_stop_required", true);
  return result;
}

std::vector<onboard::ArtifactReference> artifacts_from(const ValuePtr& value) {
  if (!value) {
    throw std::invalid_argument("onboard.manifest: artifacts is required");
  }
  std::vector<onboard::ArtifactReference> artifacts;
  for (const ValuePtr& entry : value->as_list()) {
    if (!entry || entry->kind() != Value::Kind::Map) {
      throw std::invalid_argument("onboard.manifest: artifact entries must be maps");
    }
    artifacts.push_back({entry->string_at("role"), entry->string_at("sha256")});
  }
  return artifacts;
}

Artifact build_onboard_manifest(const StageContext& context) {
  const ValuePtr interface_value = context.input->get("interface");
  onboard::DeploymentSpec specification;
  specification.target_platform = context.input->string_at("target_platform");
  specification.target_identity = target_identity_from(context.input->get("target_identity"));
  specification.model_description = context.input->string_at("model_description");
  specification.controller_description = context.input->string_at("controller_description");
  specification.failsafe_action = context.input->string_at("failsafe_action");
  specification.max_controller_time_s = context.input->number_at("max_controller_time_s");
  specification.interface = interface_from(interface_value);
  specification.transport_profile = transport_profile_from(context.input->get("transport_profile"));
  specification.artifacts = artifacts_from(context.input->get("artifacts"));

  const onboard::DeploymentPackage package = onboard::build_manifest_package(specification);
  const std::string manifest_path = context.input->string_at("manifest_path");
  const std::string checksum_path = context.input->string_at("checksum_path");
  context.write_output(manifest_path, package.manifest);
  context.write_output(checksum_path, package.manifest_sha256 + "  " + manifest_path + "\n");

  Artifact artifact;
  artifact.kind = "onboard_manifest";
  artifact.summary =
      "deterministic onboard interface manifest; qualification_state=not_qualified; "
      "executable=false; sha256="
      + package.manifest_sha256;
  artifact.payload = package;
  return artifact;
}

}  // namespace

void register_onboard_capabilities(Registry& registry) {
  registry.add(
      Capability{"onboard.manifest",
                 "Write a deterministic target-neutral onboard interface manifest and checksum",
                 "onboard_manifest",
                 Capability::State::ImplementedUnvalidated,
                 build_onboard_manifest,
                 {"target_platform",
                  "target_identity",
                  "model_description",
                  "controller_description",
                  "failsafe_action",
                  "max_controller_time_s",
                  "interface",
                  "transport_profile",
                  "artifacts",
                  "manifest_path",
                  "checksum_path"},
                 {},
                 {"manifest_path", "checksum_path"}});
}

}  // namespace galata::pipeline
