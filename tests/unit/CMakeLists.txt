# SPDX-License-Identifier: Apache-2.0

add_executable(galata_unit_tests
  ${CMAKE_SOURCE_DIR}/tests/support/evidence_check.cpp
  test_version.cpp
  test_modeling.cpp
  test_model_io.cpp
  test_linear_graph.cpp
  test_units.cpp
  test_quaternion.cpp
  test_frames.cpp
  test_state.cpp
  test_integrator.cpp
  test_modes.cpp
  test_frequency_response.cpp
  test_csv_import.cpp
  test_static_fit.cpp
  test_margins.cpp
  test_disk_margin.cpp
  test_singular_values.cpp
  test_sensitivity.cpp
  test_jacobian.cpp
  test_input_schedule.cpp
  test_newton.cpp
  test_atmosphere.cpp
  test_flight_condition.cpp
  test_control.cpp
  test_hinfinity.cpp
  test_nonlinear.cpp
  test_linear_simulation.cpp
  test_quadrotor_model.cpp
  test_hover_trim.cpp
  test_extended_linearize.cpp
)

target_link_libraries(galata_unit_tests
  PRIVATE
    galata::core
    galata::data
    galata::identify
    galata::numerics
    galata::analyze
    galata::model
    galata::modeling
    galata::trim
    galata::linearize
    galata::synth
    galata::simulation
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
