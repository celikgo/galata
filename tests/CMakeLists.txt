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
  #
  # ProjectWorkflow gets its own, larger budget, and the figure is measured
  # rather than picked. It has twenty test methods and each spawns `galata
  # project run`, whose worker SHA-256s every loaded module including the CLI
  # itself — an eighty-three-megabyte binary in the sanitizer build, read and
  # hashed once per run, through the instrumentation. Its cost therefore scales
  # with the SIZE OF THE CLI rather than with anything it tests, and it is four
  # times its nearest sibling: on `main` at f19c562 the hosted sanitizer job
  # measured 592.91 s against this 600 s limit, a margin of 1.2 percent, while
  # ProjectLinearImport, ProjectRecovery and ProjectRoutes measured 144.37,
  # 144.46 and 114.50 s. The same test measures 222 s under the local sanitizer
  # build on Darwin/arm64, which is what confirms the mechanism rather than a
  # hosted-runner anomaly.
  #
  # A 1.2 percent margin is not a margin. Three hosted sanitizer observations of
  # the same test span 592.91 s, 600.04 s (the timeout that prompted this) and
  # 740.55 s — a 25 percent spread on a machine nobody here controls, against a
  # budget it exceeded twice. The job was failing on a wall clock rather than on
  # anything numerical.
  #
  # 1500 s is about twice the widest observation and two and a half times the
  # narrowest, which absorbs both that variance and further CLI growth while
  # still catching a genuine hang inside half an hour.
  #
  # This is a wall-clock guard on a hosted machine, NOT an error budget on a
  # computed quantity. The rule against widening a gate to make a number pass is
  # about deviations measured against a stated budget, and charter rule 8 is about
  # where an expected VALUE comes from; neither reaches a ctest timeout, because
  # no number is being let through here and no comparison is being loosened. Every
  # numerical budget in this repository is untouched. The siblings keep 600 s
  # because they measured comfortably inside it; when one of them approaches its
  # limit it gets the same treatment and the same recorded reason.
  set_tests_properties(ProjectWorkflow PROPERTIES LABELS "integration" TIMEOUT 1500)
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
