# SPDX-License-Identifier: Apache-2.0

add_executable(galata_unit_tests
  test_version.cpp
  test_units.cpp
  test_quaternion.cpp
  test_frames.cpp
  test_state.cpp
)

target_link_libraries(galata_unit_tests
  PRIVATE
    galata::core
    galata::warnings
    GTest::gtest_main
)

set_target_properties(galata_unit_tests PROPERTIES FOLDER "tests")

gtest_discover_tests(galata_unit_tests
  PROPERTIES LABELS "unit"
  DISCOVERY_TIMEOUT 30
)
