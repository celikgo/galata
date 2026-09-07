// SPDX-License-Identifier: Apache-2.0
// No constructors, loader registrations or dependencies that prevent unloading.
extern "C" int galata_runtime_module_fixture() noexcept;

extern "C" int galata_runtime_module_fixture() noexcept {
  return 1;
}
