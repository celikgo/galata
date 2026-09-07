// SPDX-License-Identifier: Apache-2.0
// Loader inventories: Apple dyld(3), Linux dl_iterate_phdr(3), and Windows
// Tool Help module snapshots. Sorted paths and loader UUIDs exclude ASLR state.
// WHAT THIS IS NOT: attestation of loaded memory. An OS shared-cache/virtual
// image has no independently readable file; report that gap explicitly instead
// of inventing a digest. Differences visible between the two endpoint snapshots
// invalidate completion. A load/unload or replacement/restoration wholly between
// those endpoints is not detected; this is not continuous runtime monitoring.
#include "runtime_identity.hpp"

#include "galata/pipeline/files.hpp"

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

// Tool Help declarations require the Win32 base types from windows.h first.
#include <tlhelp32.h>
#else
#include <sys/utsname.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#include <mach-o/loader.h>

#include <dlfcn.h>
#else
#include <link.h>
#endif
#endif

namespace galata::pipeline {
namespace {
#ifdef __APPLE__
std::string image_uuid(const mach_header* header) {
  if (header == nullptr || header->magic != MH_MAGIC_64) {
    throw std::runtime_error("runtime identity: unsupported Mach-O header");
  }
  const auto* command = reinterpret_cast<const load_command*>(reinterpret_cast<const char*>(header)
                                                              + sizeof(mach_header_64));
  const auto* end = reinterpret_cast<const char*>(command) + header->sizeofcmds;
  for (std::uint32_t index = 0; index < header->ncmds; ++index) {
    if (reinterpret_cast<const char*>(command) + sizeof(load_command) > end
        || command->cmdsize < sizeof(load_command)
        || reinterpret_cast<const char*>(command) + command->cmdsize > end) {
      throw std::runtime_error("runtime identity: invalid Mach-O load commands");
    }
    if (command->cmd == LC_UUID && command->cmdsize >= sizeof(uuid_command)) {
      const auto* uuid = reinterpret_cast<const uuid_command*>(command);
      std::ostringstream result;
      result << std::hex << std::setfill('0');
      for (const auto byte : uuid->uuid) {
        result << std::setw(2) << static_cast<unsigned>(byte);
      }
      return result.str();
    }
    command = reinterpret_cast<const load_command*>(reinterpret_cast<const char*>(command)
                                                    + command->cmdsize);
  }
  return {};
}

struct AppleModuleInventory {
  std::mutex mutex;
  std::map<const mach_header*, RuntimeModule> modules;
  std::atomic<bool> failed{false};
  void* callback_owner_handle = nullptr;
};

AppleModuleInventory& apple_inventory() {
  // dyld callbacks can outlive ordinary static destructors. Keep this small
  // registry for the process lifetime rather than expose destroyed C++ state
  // during shutdown. Initialization occurs before callbacks are registered.
  static auto* inventory = new AppleModuleInventory;
  return *inventory;
}

void* pin_callback_owner() {
  // dyld has no callback unregister operation. Keeping the registry alive is
  // insufficient if an embedding application dlcloses the dylib containing
  // these callback functions. Locate this translation unit through local data
  // (not an interposable exported symbol) and retain its image for process life.
  static char owner_anchor;
  Dl_info owner{};
  if (dladdr(&owner_anchor, &owner) == 0 || owner.dli_fname == nullptr
      || owner.dli_fbase == nullptr) {
    throw std::runtime_error("runtime identity: cannot identify callback owner");
  }
  const auto* header = static_cast<const mach_header*>(owner.dli_fbase);
  if (header->magic != MH_MAGIC_64) {
    throw std::runtime_error("runtime identity: unsupported callback owner image");
  }
  if (header->filetype == MH_EXECUTE) {
    return nullptr;  // The process already owns the main executable's lifetime.
  }
  // RTLD_NODELETE is the documented dyld process-lifetime retention contract.
  // Also retain the handle: there is deliberately no matching dlclose, even at
  // static destruction, because the loader may still invoke our callbacks.
  auto* handle = dlopen(owner.dli_fname, RTLD_LAZY | RTLD_LOCAL | RTLD_NODELETE);
  if (handle == nullptr) {
    throw std::runtime_error("runtime identity: cannot retain callback owner image");
  }
  return handle;
}

void image_added(const mach_header* header, std::intptr_t) noexcept {
  auto& inventory = apple_inventory();
  try {
    if (header != nullptr && header->filetype == MH_EXECUTE) {
      return;  // The run manifest snapshots/verifies the executable separately.
    }
    // The add/remove callbacks synchronize with the loader. Copy its borrowed
    // name and mapped UUID while this image is present; count/index iteration
    // would race another thread's dlopen/dlclose and cannot pin these pointers.
    // Never call back into the loader while holding our own inventory mutex.
    Dl_info info{};
    if (dladdr(header, &info) == 0 || info.dli_fname == nullptr) {
      throw std::runtime_error("runtime identity: loaded image has no path");
    }
    RuntimeModule module;
    module.path = info.dli_fname;
    module.loader_identity = image_uuid(header);
    const std::lock_guard lock(inventory.mutex);
    inventory.modules.insert_or_assign(header, std::move(module));
  } catch (...) {
    // Allocation/metadata errors must never unwind through the C loader.
    inventory.failed.store(true);
  }
}

void image_removed(const mach_header* header, std::intptr_t) noexcept {
  auto& inventory = apple_inventory();
  try {
    const std::lock_guard lock(inventory.mutex);
    inventory.modules.erase(header);
  } catch (...) {
    inventory.failed.store(true);
  }
}

std::vector<RuntimeModule> apple_modules() {
  auto& inventory = apple_inventory();
  static std::once_flag registration;
  std::call_once(registration, [&inventory] {
    inventory.callback_owner_handle = pin_callback_owner();
    // Install removals first. Registering additions then synchronously reports
    // every currently loaded image, covering additions in the intervening gap.
    _dyld_register_func_for_remove_image(image_removed);
    _dyld_register_func_for_add_image(image_added);
  });
  const std::lock_guard lock(inventory.mutex);
  if (inventory.failed.load()) {
    throw std::runtime_error("runtime identity: loader inventory callback failed");
  }
  std::vector<RuntimeModule> modules;
  modules.reserve(inventory.modules.size());
  for (const auto& [header, module] : inventory.modules) {
    (void)header;  // Addresses are registry keys only, never serialized identity.
    modules.push_back(module);
  }
  return modules;
}
#elif !defined(_WIN32)
int collect_module(dl_phdr_info* info, std::size_t, void* data) {
  // Never let an allocation failure unwind through the C loader callback.
  auto* state = static_cast<std::pair<std::vector<RuntimeModule>, bool>*>(data);
  try {
    const std::string path = info->dlpi_name;
    if (!path.empty()) {
      RuntimeModule module;
      module.path = path;
      state->first.push_back(std::move(module));
    }
  } catch (...) {
    state->second = true;
    return 1;
  }
  return 0;
}
#endif
}  // namespace

RuntimeIdentity snapshot_runtime(bool hash_files) {
  RuntimeIdentity result;
#ifdef _WIN32
  // The actual Windows system DLL byte hashes below carry the OS binary
  // identity; avoid version APIs whose value is affected by app manifests.
  result.os_identity = "Windows; system module identities recorded below";
  HANDLE snapshot = INVALID_HANDLE_VALUE;
  // The documented ERROR_BAD_LENGTH can be transient during module changes.
  // Retry only that condition, with a fixed bound so a busy loader cannot hang
  // a study indefinitely. Other errors retain their immediate refusal.
  constexpr unsigned kSnapshotAttempts = 8;
  for (unsigned attempt = 0; attempt < kSnapshotAttempts; ++attempt) {
    snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, GetCurrentProcessId());
    if (snapshot != INVALID_HANDLE_VALUE || GetLastError() != ERROR_BAD_LENGTH) {
      break;
    }
  }
  if (snapshot == INVALID_HANDLE_VALUE) {
    throw std::runtime_error("runtime identity: cannot enumerate loaded modules");
  }
  MODULEENTRY32W entry{};
  entry.dwSize = sizeof(entry);
  if (Module32FirstW(snapshot, &entry) == FALSE) {
    CloseHandle(snapshot);
    throw std::runtime_error("runtime identity: cannot read loaded modules");
  }
  try {
    do {
      if (entry.hModule == GetModuleHandleW(nullptr)) {
        continue;  // Separately recorded as the run's executable identity.
      }
      RuntimeModule module;
      module.path = std::filesystem::path(entry.szExePath).string();
      result.modules.push_back(std::move(module));
    } while (Module32NextW(snapshot, &entry) != FALSE);
    if (GetLastError() != ERROR_NO_MORE_FILES) {
      throw std::runtime_error("runtime identity: incomplete loaded module enumeration");
    }
  } catch (...) {
    CloseHandle(snapshot);
    throw;
  }
  CloseHandle(snapshot);
#else
  utsname identity{};
  if (uname(&identity) != 0) {
    throw std::runtime_error("runtime identity: cannot identify OS");
  }
  result.os_identity = std::string(identity.sysname) + " " + identity.release + " "
                       + identity.version + " " + identity.machine;
#ifdef __APPLE__
  result.modules = apple_modules();
#else
  std::pair<std::vector<RuntimeModule>, bool> state;
  if (dl_iterate_phdr(collect_module, &state) != 0 || state.second) {
    throw std::runtime_error("runtime identity: cannot enumerate loaded modules");
  }
  result.modules = std::move(state.first);
#endif
#endif
  for (auto& module : result.modules) {
#if !defined(_WIN32) && !defined(__APPLE__)
    if (module.path == "linux-vdso.so.1" || module.path == "linux-gate.so.1") {
      module.storage = "kernel_virtual";
      continue;
    }
#endif
#ifdef __APPLE__
    if ((module.path.starts_with("/usr/lib/") || module.path.starts_with("/System/Library/"))
        && !std::filesystem::exists(module.path)) {
      // Modern macOS may provide system images solely in the dyld shared cache.
      // A loader UUID + OS build identifies them, but is not a byte digest.
      if (module.loader_identity.empty()) {
        throw std::runtime_error("runtime identity: unavailable system image has no UUID");
      }
      module.storage = "os_managed";
      continue;
    }
#endif
    module.storage = "file";
    if (hash_files) {
      const auto file = snapshot_file(module.path);
      module.path = file.path;
      module.sha256 = file.sha256;
      module.size_bytes = file.bytes.size();
    } else {
      module.path = std::filesystem::canonical(module.path).string();
      module.size_bytes = std::filesystem::file_size(module.path);
    }
  }
  std::sort(result.modules.begin(), result.modules.end(), [](const auto& left, const auto& right) {
    return left.path < right.path;
  });
  if (result.modules.empty()) {
    throw std::runtime_error("runtime identity: no loaded modules identified");
  }
  return result;
}

void verify_runtime_unchanged(const RuntimeIdentity& before) {
  if (snapshot_runtime() != before) {
    throw std::runtime_error("runtime identity changed during study; completion manifest refused");
  }
}
}  // namespace galata::pipeline
