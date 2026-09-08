# SPDX-License-Identifier: Apache-2.0
#
# Test tiers, per docs/TESTING.md:
#   unit/        one algorithm at a time, against closed-form answers
#   property/    invariants over many generated inputs, seeded and reproducible
#   integration/ one pipeline stage at a time, against a frozen contract
#   validation/  against published reference data (docs/VERIFICATION.md)
#   determinism/ bit-identical repeat runs, and cross-platform agreement
#
# A directory appears here in the same commit that adds its first test, never
# before.

find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_subdirectory(unit)
add_subdirectory(property)
add_subdirectory(integration)
add_subdirectory(validation)
add_subdirectory(determinism)

if(APPLE AND TARGET galata_desktop)
  add_subdirectory(desktop)
endif()

if(UNIX AND TARGET galata_cli)
  find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
  add_test(NAME ProjectWorkflow
    COMMAND "${CMAKE_COMMAND}" -E env "GALATA_PROJECT_CLI=$<TARGET_FILE:galata_cli>"
      "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/scripts/test_project_workflow.py")
  # Instrumented workers hash their full executable/runtime inventory for every
  # run; give the process/recovery suite room on sanitizer and hosted machines.
  set_tests_properties(ProjectWorkflow PROPERTIES LABELS "integration" TIMEOUT 600)
  add_test(NAME ProjectLinearImport
    COMMAND "${CMAKE_COMMAND}" -E env "GALATA_PROJECT_CLI=$<TARGET_FILE:galata_cli>"
      "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/scripts/test_project_linear_import.py")
  set_tests_properties(ProjectLinearImport PROPERTIES LABELS "integration" TIMEOUT 600)
  add_test(NAME ProjectRecovery
    COMMAND "${CMAKE_COMMAND}" -E env "GALATA_PROJECT_CLI=$<TARGET_FILE:galata_cli>"
      "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/scripts/test_project_recovery.py")
  set_tests_properties(ProjectRecovery PROPERTIES LABELS "integration" TIMEOUT 600)
  add_test(NAME ProjectRoutes
    COMMAND "${CMAKE_COMMAND}" -E env "GALATA_PROJECT_CLI=$<TARGET_FILE:galata_cli>"
      "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/scripts/test_project_routes.py")
  set_tests_properties(ProjectRoutes PROPERTIES LABELS "integration" TIMEOUT 600)
  add_test(NAME DesktopPackageContract
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/scripts/test_desktop_packaging.py")
  set_tests_properties(DesktopPackageContract PROPERTIES LABELS "integration" TIMEOUT 60)
endif()
