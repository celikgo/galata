# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.24)

# ---------------------------------------------------------------------------
# Version: single source of truth (docs/adr/0005-single-source-of-version.md).
# The VERSION file is the only place a version number is written by hand.
# CMake, vcpkg.json, the generated build_config.h, the CLI --version output
# and the desktop About box all derive from it, and
# scripts/check-version-consistency.sh fails CI when any of them disagree.
# ---------------------------------------------------------------------------
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" GALATA_VERSION_RAW)
string(STRIP "${GALATA_VERSION_RAW}" GALATA_VERSION)

project(galata
  VERSION ${GALATA_VERSION}
  DESCRIPTION "Flight dynamics, control-law design and simulation."
  LANGUAGES C CXX
)

# Plugin ABI major version is independent of the project version and is pinned
# for the life of the 1.x series. See docs/adr/0001-independent-c-abi.md.
set(GALATA_ABI_VERSION 1 CACHE STRING "galata plugin ABI major version" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(GNUInstallDirs)
include(CTest)
include(GalataOptions)
include(GalataWarnings)
include(GalataDeterminism)

# Public include search path. Generated headers land under build/generated.
add_library(galata_public_headers INTERFACE)
target_include_directories(galata_public_headers
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
add_library(galata::public_headers ALIAS galata_public_headers)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/galata/build_config.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/generated/include/galata/build_config.hpp"
  @ONLY
)

add_subdirectory(src)

# Operator-facing binaries that pair with the runtime libraries.
add_subdirectory(tools/validation)

if(BUILD_TESTING AND GALATA_BUILD_TESTS)
  add_subdirectory(tests)
endif()

message(STATUS "")
message(STATUS "galata configuration")
message(STATUS "  Version:        ${PROJECT_VERSION}")
message(STATUS "  ABI version:    ${GALATA_ABI_VERSION}")
message(STATUS "  Build type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard:   ${CMAKE_CXX_STANDARD}")
message(STATUS "  Compiler:       ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  FP contraction: off (determinism policy, ADR-0004)")
message(STATUS "  Tests:          ${GALATA_BUILD_TESTS}")
message(STATUS "  Werror:         ${GALATA_WERROR}")
message(STATUS "")
