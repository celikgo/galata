// SPDX-License-Identifier: Apache-2.0

#include "galata/version.hpp"

#include "galata/build_config.hpp"

#include <string>

namespace galata {

namespace {

// Assembled once at static-init time rather than per call, so that
// build_identification() can return a string_view into storage that outlives
// every caller.
const std::string build_id_storage =  // NOLINT(cert-err58-cpp)
    std::string("galata ") + GALATA_VERSION_STRING + " (" + GALATA_BUILD_COMPILER_ID + " "
    + GALATA_BUILD_COMPILER_VERSION + ", " + GALATA_BUILD_TYPE + ", " + GALATA_BUILD_SYSTEM + "/"
    + GALATA_BUILD_PROCESSOR + ")";

}  // namespace

std::string_view version_string() noexcept {
  return GALATA_VERSION_STRING;
}

int version_major() noexcept {
  return GALATA_VERSION_MAJOR;
}

int version_minor() noexcept {
  return GALATA_VERSION_MINOR;
}

int version_patch() noexcept {
  return GALATA_VERSION_PATCH;
}

int abi_version_major() noexcept {
  return GALATA_ABI_VERSION_MAJOR_CONFIGURED;
}

std::string_view build_identification() noexcept {
  return build_id_storage;
}

}  // namespace galata
