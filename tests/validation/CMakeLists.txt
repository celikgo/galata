# SPDX-License-Identifier: Apache-2.0
#
# Validation: output against PUBLISHED reference data (charter rule 8).
#
# Distinct from the unit tier in what a failure means. A unit-test failure says
# the code does not do what the author intended. A validation failure says the
# code does not agree with a document that a stranger can go and read.
#
# Every case cites a file in reference/, and every file in reference/ carries a
# citation header naming the document, its report number, its rights position
# and how the values were transcribed.

# The reference data is read at run time rather than compiled in, so that a
# reviewer can diff the transcription against the source document without
# recompiling, and so the citation header travels with the numbers.
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/validation_config.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/validation_config.hpp"
  @ONLY
)

add_executable(galata_validation_tests
  ${CMAKE_SOURCE_DIR}/tests/support/evidence_check.cpp
  test_atmosphere_ussa1976.cpp
  test_rigid_body_dynamics.cpp
  test_nt33a_modes.cpp
  test_nt33a_trim_linearize.cpp
  test_disk_margin_seiler.cpp
  test_case_registry.cpp
)

target_include_directories(galata_validation_tests
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
)

target_link_libraries(galata_validation_tests
  PRIVATE
    galata::core
    galata::numerics
    galata::sim
    galata::analyze
    galata::trim
    galata::linearize
    galata::validation_reference
    galata::pipeline
    galata::warnings
    GTest::gtest_main
)

target_compile_definitions(galata_validation_tests
  PRIVATE GALATA_TEST_BINARY=galata::validation::Binary::Validation
)

target_link_libraries(galata_validation_tests PRIVATE galata::validation_case_check)

set_target_properties(galata_validation_tests PROPERTIES FOLDER "tests")

gtest_discover_tests(galata_validation_tests
  PROPERTIES LABELS "validation"
  DISCOVERY_TIMEOUT 30
)
