// SPDX-License-Identifier: Apache-2.0
#include "galata/onboard/controller_plugin.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <dlfcn.h>

namespace galata::onboard {
namespace {

std::string read_manifest(const std::string& manifest) {
  if (manifest.size() > 2U * 1024U * 1024U) {
    throw std::invalid_argument("onboard: controller manifest exceeds the 2 MiB limit");
  }
  return manifest;
}

template <typename Function>
Function load_function(void* handle, const char* name) {
  (void)::dlerror();
  void* symbol = ::dlsym(handle, name);
  const char* error = ::dlerror();
  if (error != nullptr || symbol == nullptr) {
    throw std::invalid_argument(std::string("onboard: controller is missing symbol '") + name
                                + "'");
  }
  static_assert(sizeof(Function) == sizeof(symbol),
                "POSIX function and object pointers must have equal representation here");
  Function function = nullptr;
  std::memcpy(&function, &symbol, sizeof(function));
  return function;
}

std::string loader_error(const char* operation, const std::filesystem::path& path) {
  const char* detail = ::dlerror();
  return std::string("onboard: controller ") + operation + " '" + path.string() + "'"
         + (detail == nullptr ? std::string{} : std::string(": ") + detail);
}

}  // namespace

ControllerPlugin::ControllerPlugin(const std::filesystem::path& library,
                                   const std::filesystem::path& model,
                                   const hardware::InterfaceSpec& interface,
                                   const std::string& manifest)
    : library_(library), model_(model), actuator_count_(interface.actuator_channels.size()) {
  hardware::validate_interface(interface);
  if (library_.empty() || std::filesystem::is_symlink(library_)
      || !std::filesystem::is_regular_file(library_)) {
    throw std::invalid_argument("onboard: controller library must be a regular non-symlink file: "
                                + library_.string());
  }
  if (model_.empty() || std::filesystem::is_symlink(model_)
      || !std::filesystem::is_regular_file(model_)) {
    throw std::invalid_argument("onboard: controller model must be a regular non-symlink file: "
                                + model_.string());
  }
  if (actuator_count_ == 0U) {
    throw std::invalid_argument("onboard: controller requires at least one actuator channel");
  }
  const std::string bounded_manifest = read_manifest(manifest);

  (void)::dlerror();
  handle_ = ::dlopen(library_.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (handle_ == nullptr) {
    throw std::invalid_argument(loader_error("load", library_));
  }

  try {
    const GalataControllerAbiVersionFn version =
        load_function<GalataControllerAbiVersionFn>(handle_, "galata_controller_abi_version");
    if (version() != GALATA_CONTROLLER_ABI_VERSION) {
      throw std::invalid_argument("onboard: controller ABI version is incompatible");
    }
    const GalataControllerCreateFn create =
        load_function<GalataControllerCreateFn>(handle_, "galata_controller_create");
    step_function_ = load_function<GalataControllerStepFn>(handle_, "galata_controller_step");
    destroy_function_ =
        load_function<GalataControllerDestroyFn>(handle_, "galata_controller_destroy");
    const std::string model_path = model_.string();
    context_ = create(bounded_manifest.data(),
                      bounded_manifest.size(),
                      model_path.data(),
                      model_path.size(),
                      interface.sensor_channels.size(),
                      interface.actuator_channels.size());
    if (context_ == nullptr) {
      throw std::invalid_argument("onboard: controller refused its declared interface");
    }
  } catch (...) {
    close();
    throw;
  }
}

ControllerPlugin::~ControllerPlugin() {
  close();
}

ControllerPlugin::ControllerPlugin(ControllerPlugin&& other) noexcept
    : library_(std::move(other.library_)), model_(std::move(other.model_)),
      handle_(std::exchange(other.handle_, nullptr)),
      context_(std::exchange(other.context_, nullptr)),
      step_function_(std::exchange(other.step_function_, nullptr)),
      destroy_function_(std::exchange(other.destroy_function_, nullptr)),
      actuator_count_(std::exchange(other.actuator_count_, 0U)) {}

ControllerPlugin& ControllerPlugin::operator=(ControllerPlugin&& other) noexcept {
  if (this != &other) {
    close();
    library_ = std::move(other.library_);
    model_ = std::move(other.model_);
    handle_ = std::exchange(other.handle_, nullptr);
    context_ = std::exchange(other.context_, nullptr);
    step_function_ = std::exchange(other.step_function_, nullptr);
    destroy_function_ = std::exchange(other.destroy_function_, nullptr);
    actuator_count_ = std::exchange(other.actuator_count_, 0U);
  }
  return *this;
}

hardware::Frame ControllerPlugin::step(const hardware::Frame& sensor) {
  if (handle_ == nullptr || context_ == nullptr || step_function_ == nullptr) {
    throw std::logic_error("onboard: controller plugin is not loaded");
  }
  if (sensor.values.empty()) {
    throw std::invalid_argument("onboard: controller plugin received an empty sensor frame");
  }
  std::vector<double> values(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  const GalataControllerSensorFrame input{
      sensor.sequence, sensor.timestamp_s, sensor.values.size(), sensor.values.data()};
  GalataControllerActuatorFrame output{values.size(), values.data()};
  char diagnostic[512]{};
  const int result = step_function_(context_, &input, &output, diagnostic, sizeof(diagnostic));
  if (result != 0) {
    const std::string detail = diagnostic[0] == '\0' ? "controller callback failed" : diagnostic;
    throw std::runtime_error("onboard: " + detail);
  }
  return hardware::Frame{sensor.sequence, sensor.timestamp_s, std::move(values)};
}

void ControllerPlugin::close() noexcept {
  if (context_ != nullptr && destroy_function_ != nullptr) {
    destroy_function_(context_);
  }
  context_ = nullptr;
  destroy_function_ = nullptr;
  step_function_ = nullptr;
  if (handle_ != nullptr) {
    (void)::dlclose(handle_);
    handle_ = nullptr;
  }
}

}  // namespace galata::onboard
