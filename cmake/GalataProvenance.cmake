# SPDX-License-Identifier: Apache-2.0
# Invoked at build time. Missing Git metadata is an explicit unknown, not a
# fabricated clean revision. configure_file changes the header only on change.
set(GALATA_SOURCE_COMMIT "unknown")
set(GALATA_SOURCE_STATUS "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --show-toplevel
    WORKING_DIRECTORY "${GALATA_SOURCE_DIR}"
    RESULT_VARIABLE top_result OUTPUT_VARIABLE git_top OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
    WORKING_DIRECTORY "${GALATA_SOURCE_DIR}"
    RESULT_VARIABLE git_result OUTPUT_VARIABLE revision OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(git_result EQUAL 0 AND top_result EQUAL 0 AND git_top STREQUAL GALATA_SOURCE_DIR)
    set(GALATA_SOURCE_COMMIT "${revision}")
    execute_process(COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=normal
      WORKING_DIRECTORY "${GALATA_SOURCE_DIR}"
      RESULT_VARIABLE status_result OUTPUT_VARIABLE changes OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    if(status_result EQUAL 0)
      if(changes STREQUAL "")
        set(GALATA_SOURCE_STATUS "clean")
      else()
        set(GALATA_SOURCE_STATUS "dirty")
      endif()
    endif()
  endif()
endif()
file(SHA256 "${GALATA_SOURCE_DIR}/vcpkg.json" GALATA_DEPENDENCY_MANIFEST_SHA256)
find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${GALATA_SOURCE_DIR}/scripts/build-provenance.py"
    "${GALATA_SOURCE_DIR}" "${GALATA_BINARY_DIR}"
  RESULT_VARIABLE provenance_result OUTPUT_VARIABLE provenance_identity
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT provenance_result EQUAL 0)
  message(FATAL_ERROR "Cannot identify current source/build configuration")
endif()
list(GET provenance_identity 0 GALATA_SOURCE_TREE_SHA256)
list(GET provenance_identity 1 GALATA_BUILD_CONFIGURATION_SHA256)
file(READ "${GALATA_BINARY_DIR}/galata-build-configuration.json" GALATA_BUILD_CONFIGURATION_JSON)
string(STRIP "${GALATA_BUILD_CONFIGURATION_JSON}" GALATA_BUILD_CONFIGURATION_JSON)
# Preserve canonical JSON as a single C++ string literal.
string(REPLACE "\\" "\\\\" GALATA_BUILD_CONFIGURATION_JSON "${GALATA_BUILD_CONFIGURATION_JSON}")
string(REPLACE "\"" "\\\"" GALATA_BUILD_CONFIGURATION_JSON "${GALATA_BUILD_CONFIGURATION_JSON}")
configure_file("${GALATA_SOURCE_DIR}/src/pipeline/provenance_config.hpp.in"
  "${GALATA_PROVENANCE_HEADER}" @ONLY)
