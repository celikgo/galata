// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_PIPELINE_RUNTIME_IDENTITY_HPP
#define GALATA_PIPELINE_RUNTIME_IDENTITY_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace galata::pipeline {
// Identity of loaded shared modules and their on-disk bytes; the executable
// is recorded separately by the pipeline. This does not
// attest process memory or replace an authenticated release/package signature.
// Comparing start/end snapshots detects visible endpoint differences, not
// transient module/file changes that are restored entirely between snapshots.
struct RuntimeModule {
  std::string path;
  std::string storage;          // file, os_managed (shared cache), or kernel_virtual
  std::string loader_identity;  // Mach-O UUID where the loader exposes it
  std::string sha256;
  std::uintmax_t size_bytes = 0;
  bool operator==(const RuntimeModule&) const = default;
};

struct RuntimeIdentity {
  std::string os_identity;
  std::vector<RuntimeModule> modules;
  bool operator==(const RuntimeIdentity&) const = default;
};

// On macOS the first snapshot registers process-lifetime dyld callbacks. The
// containing dylib/bundle is retained with RTLD_NODELETE, so an embedding host
// cannot unload that image after this call. Registry state is also retained
// through process shutdown. A statically linked main executable needs no pin.
// hash_files=false is path protection only, for a run without a manifest; it
// does not supply byte identity and must not be used as completion evidence.
[[nodiscard]] RuntimeIdentity snapshot_runtime(bool hash_files = true);
void verify_runtime_unchanged(const RuntimeIdentity& before);
}  // namespace galata::pipeline
#endif
