// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_CORE_SHA256_HPP
#define GALATA_CORE_SHA256_HPP
#include <string>
#include <string_view>

namespace galata::core {
// NIST FIPS 180-4 SHA-256 content identity; not authentication or a signature.
[[nodiscard]] std::string sha256(std::string_view bytes);
}  // namespace galata::core
#endif
