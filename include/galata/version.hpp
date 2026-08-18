// SPDX-License-Identifier: Apache-2.0
//
// Runtime version accessors.
//
// The CLI's --version output, the Python binding's __version__ and the desktop
// About box all read these functions, so there is exactly one version number in
// the product and scripts/check-version-consistency.sh can prove it.

#ifndef GALATA_VERSION_HPP
#define GALATA_VERSION_HPP

#include <string_view>

namespace galata {

// Semantic version of this build, e.g. "0.0.1". Derived from the VERSION file
// at the repository root at configure time.
[[nodiscard]] std::string_view version_string() noexcept;

[[nodiscard]] int version_major() noexcept;
[[nodiscard]] int version_minor() noexcept;
[[nodiscard]] int version_patch() noexcept;

// Plugin C ABI major version this host implements.
[[nodiscard]] int abi_version_major() noexcept;

// One-line build provenance, e.g.
//   "galata 0.0.1 (AppleClang 21.0.0, RelWithDebInfo, Darwin/arm64)"
// Recorded in every result file so a number can be traced to the binary that
// produced it.
[[nodiscard]] std::string_view build_identification() noexcept;

}  // namespace galata

#endif  // GALATA_VERSION_HPP
