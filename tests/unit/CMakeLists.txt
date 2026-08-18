# SPDX-License-Identifier: Apache-2.0

add_executable(galata_unit_tests
  ${CMAKE_SOURCE_DIR}/tests/support/evidence_check.cpp
  test_version.cpp
  test_units.cpp
  test_quaternion.cpp
  test_frames.cpp
  test_state.cpp
  test_integrator.cpp
  test_modes.cpp
  test_jacobian.cpp
  test_newton.cpp
)

target_link_libraries(galata_unit_tests
  PRIVATE
    galata::core
    galata::numerics
    galata::analyze
    galata::warnings
    GTest::gtest_main
)

target_compile_definitions(galata_unit_tests
  PRIVATE GALATA_TEST_BINARY=galata::validation::Binary::Unit
)

target_link_libraries(galata_unit_tests PRIVATE galata::validation_case_check)

set_target_properties(galata_unit_tests PROPERTIES FOLDER "tests")

gtest_discover_tests(galata_unit_tests
  PROPERTIES LABELS "unit"
  DISCOVERY_TIMEOUT 30
)
