# SPDX-License-Identifier: Apache-2.0
#
# Property tests: invariants asserted over many generated inputs.
#
# No property-testing framework is used. The generators are a seeded
# std::mt19937_64 and hand-written sampling, for the reason in ADR-0004: a
# framework that shrinks a counterexample using its own PRNG makes the failing
# case a function of the framework's version. A fixed seed printed in the
# failure message is reproducible by anyone, forever.

add_executable(galata_property_tests
  property_generators.cpp
  test_quaternion_properties.cpp
  test_frame_properties.cpp
  test_state_properties.cpp
)

target_link_libraries(galata_property_tests
  PRIVATE
    galata::core
    galata::warnings
    GTest::gtest_main
)

set_target_properties(galata_property_tests PROPERTIES FOLDER "tests")

gtest_discover_tests(galata_property_tests
  PROPERTIES LABELS "property"
  DISCOVERY_TIMEOUT 30
)
