
# The ULog fixture is generated rather than committed: a binary blob nobody can
# read is a fixture nobody can check. Writing it needs only Python; the
# independent check against pyulog is a separate step, so this does not put a
# third-party parser in the build.
find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
set(GALATA_ULOG_FIXTURE "${CMAKE_CURRENT_BINARY_DIR}/sample.ulg")
add_custom_command(
  OUTPUT "${GALATA_ULOG_FIXTURE}"
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/data/make_ulog_fixture.py"
          "${GALATA_ULOG_FIXTURE}"
  DEPENDS "${CMAKE_SOURCE_DIR}/tests/data/make_ulog_fixture.py"
  COMMENT "Generating the ULog fixture"
  VERBATIM)
add_custom_target(galata_ulog_fixture DEPENDS "${GALATA_ULOG_FIXTURE}")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/ulog_fixture.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/ulog_fixture.hpp"
  @ONLY)

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
  test_ulog_import.cpp
  test_chart_mapping.cpp
  test_greybox.cpp
  test_validate.cpp
  test_static_fit.cpp
  test_record_window.cpp
  test_quadrotor_serialisation.cpp
  test_gramians.cpp
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
add_dependencies(galata_unit_tests galata_ulog_fixture)
target_include_directories(galata_unit_tests PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")

set_target_properties(galata_unit_tests PROPERTIES FOLDER "tests")

gtest_discover_tests(galata_unit_tests
  PROPERTIES LABELS "unit"
  DISCOVERY_TIMEOUT 30
)
