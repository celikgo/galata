# SPDX-License-Identifier: Apache-2.0
#
# Determinism, ADR-0004 tier 1: same binary, same platform, bit-identical.
#
# In-process here; the cross-platform tier 2 comparison lives in
# .github/workflows/determinism.yml, because it needs three machines.

add_executable(galata_determinism_tests
  ${CMAKE_SOURCE_DIR}/tests/support/evidence_check.cpp
  test_bit_identical.cpp
)

target_link_libraries(galata_determinism_tests
  PRIVATE
    galata::core
    galata::numerics
    galata::sim
    galata::analyze
    galata::warnings
    GTest::gtest_main
)

target_compile_definitions(galata_determinism_tests
  PRIVATE GALATA_TEST_BINARY=galata::validation::Binary::Determinism
)

target_link_libraries(galata_determinism_tests PRIVATE galata::validation_case_check)

set_target_properties(galata_determinism_tests PROPERTIES FOLDER "tests")

gtest_discover_tests(galata_determinism_tests
  PROPERTIES LABELS "determinism"
  DISCOVERY_TIMEOUT 30
)
