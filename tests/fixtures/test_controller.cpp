// SPDX-License-Identifier: Apache-2.0
#include "galata/onboard/controller_abi.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>

namespace {

struct Context {
  double gain = 0.001;
};

int diagnostic(char* output, std::size_t capacity, const char* message) {
  if (output != nullptr && capacity != 0U) {
    std::strncpy(output, message, capacity - 1U);
    output[capacity - 1U] = '\0';
  }
  return 1;
}

}  // namespace

extern "C" std::uint32_t galata_controller_abi_version() {
  return GALATA_CONTROLLER_ABI_VERSION;
}

extern "C" void* galata_controller_create(const char* manifest,
                                          std::size_t manifest_size,
                                          const char* model_path,
                                          std::size_t model_path_size,
                                          std::size_t sensor_count,
                                          std::size_t actuator_count) {
  if (manifest == nullptr || manifest_size == 0U || model_path == nullptr || model_path_size == 0U
      || sensor_count != 1U || actuator_count != 1U) {
    return nullptr;
  }
  return new (std::nothrow) Context{};
}

extern "C" int galata_controller_step(void* opaque,
                                      const GalataControllerSensorFrame* sensor,
                                      GalataControllerActuatorFrame* actuator,
                                      char* error,
                                      std::size_t error_size) {
  if (opaque == nullptr || sensor == nullptr || actuator == nullptr || sensor->values == nullptr
      || actuator->values == nullptr || sensor->value_count != 1U || actuator->value_count != 1U) {
    return diagnostic(error, error_size, "test controller received the wrong channel width");
  }
  if (!std::isfinite(sensor->values[0])) {
    return diagnostic(error, error_size, "test controller received a non-finite value");
  }
  const auto* context = static_cast<const Context*>(opaque);
  actuator->values[0] = sensor->values[0] * context->gain;
  return 0;
}

extern "C" void galata_controller_destroy(void* opaque) {
  delete static_cast<Context*>(opaque);
}
