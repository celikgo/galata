# SPDX-License-Identifier: Apache-2.0
#
# Integration: one pipeline stage at a time, against its documented contract.
#
# Written WITHOUT reading the implementation (docs/TESTING.md). A test written
# by reading the code tests the code's opinion of itself.

find_package(yaml-cpp CONFIG REQUIRED)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/integration_config.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/integration_config.hpp"
  @ONLY
)

add_executable(galata_integration_tests
  ${CMAKE_SOURCE_DIR}/tests/support/evidence_check.cpp
  test_pipeline.cpp
  test_example_studies.cpp
  test_product_hardening.cpp
  test_evidence_contract.cpp
  test_runtime_identity.cpp
  test_design_workflow.cpp
  test_model_workflow.cpp
  test_quadrotor_workflow.cpp
  test_linear_graph_workflow.cpp
  test_model_input_limits.cpp
  test_pipeline_cancellation.cpp
  test_simulation_convergence.cpp
)

target_include_directories(galata_integration_tests
  PRIVATE "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_SOURCE_DIR}/src/pipeline"
)

target_link_libraries(galata_integration_tests
  PRIVATE
    galata::pipeline
    galata::warnings
    GTest::gtest_main
    yaml-cpp::yaml-cpp
)

target_compile_definitions(galata_integration_tests
  PRIVATE GALATA_TEST_BINARY=galata::validation::Binary::Integration
)

target_link_libraries(galata_integration_tests PRIVATE galata::validation_case_check)

if(APPLE)
  # A real unloadable image exercises dyld callbacks; shared-cache OS dylibs
  # need not unmap on dlclose. This fixture also tests a callback-owning image.
  find_package(Threads REQUIRED)
  add_library(galata_runtime_module_fixture MODULE runtime_module_fixture.cpp)
  target_link_libraries(galata_runtime_module_fixture PRIVATE galata::warnings)
  add_library(galata_runtime_identity_fixture MODULE
    runtime_identity_fixture.cpp
    "${CMAKE_SOURCE_DIR}/src/pipeline/runtime_identity.cpp"
    "${CMAKE_SOURCE_DIR}/src/pipeline/files.cpp"
    "${CMAKE_SOURCE_DIR}/src/pipeline/sha256.cpp"
  )
  target_include_directories(galata_runtime_identity_fixture
    PRIVATE "${CMAKE_SOURCE_DIR}/src/pipeline"
  )
  target_link_libraries(galata_runtime_identity_fixture
    PRIVATE galata::core galata::public_headers galata::warnings Threads::Threads ${CMAKE_DL_LIBS}
  )
  target_link_libraries(galata_integration_tests PRIVATE Threads::Threads ${CMAKE_DL_LIBS})
  target_compile_definitions(galata_integration_tests PRIVATE
    GALATA_RUNTIME_IDENTITY_FIXTURE="$<TARGET_FILE:galata_runtime_identity_fixture>"
    GALATA_RUNTIME_MODULE_FIXTURE="$<TARGET_FILE:galata_runtime_module_fixture>"
  )
  add_dependencies(galata_integration_tests
    galata_runtime_identity_fixture galata_runtime_module_fixture
  )
  set_target_properties(galata_runtime_identity_fixture galata_runtime_module_fixture
    PROPERTIES FOLDER "tests"
  )
endif()

set_target_properties(galata_integration_tests PROPERTIES FOLDER "tests")

gtest_discover_tests(galata_integration_tests
  PROPERTIES LABELS "integration"
  DISCOVERY_TIMEOUT 30
)
