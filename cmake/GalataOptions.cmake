# SPDX-License-Identifier: Apache-2.0
#
# Public configuration options for the galata build.

include_guard(GLOBAL)

option(GALATA_BUILD_TESTS      "Build unit, integration, validation and determinism tests" ON)
option(GALATA_BUILD_EXAMPLES   "Build the runnable example studies"                        OFF)
option(GALATA_BUILD_BENCHMARKS "Build the benchmark suite"                                 OFF)
option(GALATA_BUILD_CLI        "Build the galata CLI executable"                           ON)
option(GALATA_BUILD_PYTHON     "Build the pygalata Python bindings"                        OFF)

option(GALATA_ENABLE_ASAN      "AddressSanitizer (debug builds)"        OFF)
option(GALATA_ENABLE_UBSAN     "UndefinedBehaviorSanitizer"             OFF)

option(GALATA_WERROR           "Treat compiler warnings as errors"      ON)

if(GALATA_ENABLE_ASAN)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address)
endif()

if(GALATA_ENABLE_UBSAN)
  add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=undefined)
endif()
