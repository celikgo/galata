# SPDX-License-Identifier: Apache-2.0
#
# Integration: one pipeline stage at a time, against its documented contract.
#
# Written WITHOUT reading the implementation (docs/TESTING.md). A test written
# by reading the code tests the code's opinion of itself.

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/integration_config.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/integration_config.hpp"
  @ONLY
)

add_executable(galata_integration_tests
  test_pipeline.cpp
  test_example_studies.cpp
)

target_include_directories(galata_integration_tests
  PRIVATE "${CMAKE_CURRENT_BINARY_DIR}"
)

target_link_libraries(galata_integration_tests
  PRIVATE
    galata::pipeline
    galata::warnings
    GTest::gtest_main
)

set_target_properties(galata_integration_tests PROPERTIES FOLDER "tests")

gtest_discover_tests(galata_integration_tests
  PROPERTIES LABELS "integration"
  DISCOVERY_TIMEOUT 30
)
