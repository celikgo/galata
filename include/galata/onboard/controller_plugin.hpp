// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_ONBOARD_CONTROLLER_PLUGIN_HPP
#define GALATA_ONBOARD_CONTROLLER_PLUGIN_HPP

#include "galata/hardware/interface.hpp"
#include "galata/onboard/controller_abi.h"

#include <cstddef>
#include <filesystem>
#include <string>

namespace galata::onboard {

// Loads a controlled native controller artifact through the versioned C ABI.
// The artifact is code, not data: deployment must establish its provenance,
// integrity and platform compatibility before this class is used.
class ControllerPlugin final {
 public:
  ControllerPlugin(const std::filesystem::path& library,
                   const std::filesystem::path& model,
                   const hardware::InterfaceSpec& interface,
                   const std::string& manifest);
  ~ControllerPlugin();

  ControllerPlugin(const ControllerPlugin&) = delete;
  ControllerPlugin& operator=(const ControllerPlugin&) = delete;

  ControllerPlugin(ControllerPlugin&& other) noexcept;
  ControllerPlugin& operator=(ControllerPlugin&& other) noexcept;

  [[nodiscard]] hardware::Frame step(const hardware::Frame& sensor);
  [[nodiscard]] const std::filesystem::path& library() const noexcept {
    return library_;
  }
  [[nodiscard]] const std::filesystem::path& model() const noexcept {
    return model_;
  }

 private:
  void close() noexcept;

  std::filesystem::path library_;
  std::filesystem::path model_;
  void* handle_ = nullptr;
  void* context_ = nullptr;
  GalataControllerStepFn step_function_ = nullptr;
  GalataControllerDestroyFn destroy_function_ = nullptr;
  std::size_t actuator_count_ = 0;
};

}  // namespace galata::onboard

#endif  // GALATA_ONBOARD_CONTROLLER_PLUGIN_HPP
