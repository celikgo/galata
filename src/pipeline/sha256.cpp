// SPDX-License-Identifier: Apache-2.0
// Preserve the public pipeline API while sharing the dependency-neutral digest.
#include "galata/core/sha256.hpp"

#include "galata/pipeline/files.hpp"

namespace galata::pipeline {
std::string sha256(std::string_view bytes) {
  return core::sha256(bytes);
}
}  // namespace galata::pipeline
