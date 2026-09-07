// SPDX-License-Identifier: Apache-2.0
// An unloadable image until this entry point registers its own dyld callbacks.
#include "runtime_identity.hpp"

extern "C" int galata_runtime_fixture_initialize() noexcept;

extern "C" int galata_runtime_fixture_initialize() noexcept {
  try {
    return galata::pipeline::snapshot_runtime().modules.empty() ? 0 : 1;
  } catch (...) {
    return 0;
  }
}
