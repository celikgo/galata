// SPDX-License-Identifier: Apache-2.0
// Loader lifetime contract: copied endpoint inventories survive concurrent
// module removal, and macOS callback owners remain mapped after host dlclose.
#include "runtime_identity.hpp"
#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#ifdef __APPLE__
#include <array>
#include <atomic>
#include <filesystem>
#include <future>
#include <thread>
#include <utility>

#include <dlfcn.h>
#endif

namespace {
using galata::pipeline::RuntimeIdentity;
using galata::pipeline::snapshot_runtime;

void expect_complete_inventory(const RuntimeIdentity& identity) {
  EXPECT_FALSE(identity.os_identity.empty());
  EXPECT_FALSE(identity.modules.empty());
  EXPECT_TRUE(std::is_sorted(
      identity.modules.begin(), identity.modules.end(), [](const auto& left, const auto& right) {
        return left.path < right.path;
      }));
  for (const auto& module : identity.modules) {
    EXPECT_FALSE(module.path.empty());
    if (module.storage == "file") {
      EXPECT_EQ(module.sha256.size(), 64U);
      EXPECT_GT(module.size_bytes, 0U);
    } else {
      EXPECT_TRUE(module.storage == "os_managed" || module.storage == "kernel_virtual");
      EXPECT_TRUE(module.sha256.empty());
    }
  }
}

TEST(RuntimeIdentity, StableEndpointSnapshotsHaveDeterministicIdentities) {
  const auto first = snapshot_runtime();
  expect_complete_inventory(first);
  EXPECT_EQ(snapshot_runtime(), first);
  EXPECT_NO_THROW(galata::pipeline::verify_runtime_unchanged(first));
}

#ifdef __APPLE__
class Module {
 public:
  explicit Module(const char* path, int mode = RTLD_LAZY | RTLD_LOCAL)
      : handle_(dlopen(path, mode)) {}

  ~Module() {
    if (handle_ != nullptr) {
      (void)dlclose(handle_);
    }
  }

  Module(const Module&) = delete;
  Module& operator=(const Module&) = delete;

  [[nodiscard]] void* get() const {
    return handle_;
  }

  int close() {
    auto* handle = std::exchange(handle_, nullptr);
    return handle == nullptr ? -1 : dlclose(handle);
  }

 private:
  void* handle_;
};

bool contains_fixture(const RuntimeIdentity& identity, const char* fixture) {
  const auto path = std::filesystem::canonical(fixture).string();
  return std::any_of(identity.modules.begin(), identity.modules.end(), [&path](const auto& module) {
    return module.path == path;
  });
}

TEST(RuntimeIdentity, ConcurrentModuleLoadsAndUnloadsPreserveCopiedSnapshots) {
  // Exercise real add/remove callbacks before racing them. This ordinary
  // fixture is distinct from the callback-owning fixture that must stay pinned.
  ASSERT_FALSE(contains_fixture(snapshot_runtime(), GALATA_RUNTIME_MODULE_FIXTURE));
  {
    Module module(GALATA_RUNTIME_MODULE_FIXTURE);
    ASSERT_NE(module.get(), nullptr);
    EXPECT_TRUE(contains_fixture(snapshot_runtime(), GALATA_RUNTIME_MODULE_FIXTURE));
    ASSERT_EQ(module.close(), 0);
  }
  ASSERT_FALSE(contains_fixture(snapshot_runtime(), GALATA_RUNTIME_MODULE_FIXTURE));

  constexpr unsigned rounds = 8;
  constexpr unsigned loads_per_round = 32;
  std::array<std::promise<void>, rounds> starts;
  std::array<std::future<void>, rounds> ready;
  for (unsigned round = 0; round < rounds; ++round) {
    ready[round] = starts[round].get_future();
  }
  std::atomic<unsigned> completed_loads{0};
  std::atomic<bool> loader_failed{false};
  {
    std::thread loader([&] {
      for (unsigned round = 0; round < rounds; ++round) {
        ready[round].wait();
        for (unsigned load = 0; load < loads_per_round; ++load) {
          Module module(GALATA_RUNTIME_MODULE_FIXTURE);
          if (module.get() == nullptr || module.close() != 0) {
            loader_failed.store(true);
          } else {
            ++completed_loads;
          }
        }
      }
    });
    for (unsigned round = 0; round < rounds; ++round) {
      starts[round].set_value();
      // A changing loader may produce different endpoint lists. Every returned
      // list must contain complete copied identities, with no borrowed memory.
      EXPECT_NO_THROW(expect_complete_inventory(snapshot_runtime()));
    }
    loader.join();
  }
  EXPECT_FALSE(loader_failed.load());
  EXPECT_EQ(completed_loads.load(), rounds * loads_per_round);
  EXPECT_FALSE(contains_fixture(snapshot_runtime(), GALATA_RUNTIME_MODULE_FIXTURE));
}

TEST(RuntimeIdentity, CallbackOwningImageRemainsMappedAfterHostClosesIt) {
  // This fixture contains the production inventory, as if statically embedded in a
  // host plugin, and registers callbacks from that unloadable image itself.
  Module module(GALATA_RUNTIME_IDENTITY_FIXTURE);
  ASSERT_NE(module.get(), nullptr);
  using Initialize = int (*)() noexcept;
  const auto initialize =
      reinterpret_cast<Initialize>(dlsym(module.get(), "galata_runtime_fixture_initialize"));
  ASSERT_NE(initialize, nullptr);
  ASSERT_EQ(initialize(), 1);
  ASSERT_EQ(module.close(), 0);

  Module retained(GALATA_RUNTIME_IDENTITY_FIXTURE, RTLD_LAZY | RTLD_LOCAL | RTLD_NOLOAD);
  ASSERT_NE(retained.get(), nullptr) << "dyld callbacks must not outlive their mapped code";
  EXPECT_EQ(initialize(), 1);
  EXPECT_TRUE(contains_fixture(snapshot_runtime(), GALATA_RUNTIME_IDENTITY_FIXTURE));
}
#endif
}  // namespace
