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

  # THE PER-INVOCATION DEADLINE, and why the instrumented build cannot share
  # the uninstrumented one.
  #
  # Each of the four suites below wraps every `galata project ...` call in a
  # Python-level deadline, so that a wedged worker fails naming its own command
  # instead of arriving as an opaque ctest kill of the whole module. That
  # deadline is 45 s, set in the scripts, and 45 s was never a sound figure for
  # the sanitizer build. It survived only while the CLI stayed small enough.
  #
  # One invocation, measured on Darwin/arm64 with
  #   /usr/bin/time -p <cli> project <command> <scratch fixture>
  # against the CLI built by this repository's own `asan` preset at two commits:
  #
  #     sanitizer CLI            bytes    import-linear    project run
  #     main, ed63f76       82 882 000         13.19 s        12.05 s
  #     this branch         96 776 528         13.89 s        13.68 s
  #
  # Two readings matter here. First, the two commands do entirely different work
  # — one imports and rewrites a study, the other integrates a simulation — and
  # cost the same to within a second. That is the mechanism the ProjectWorkflow
  # note below describes, measured directly rather than inferred: the cost is
  # dominated by the worker hashing its own executable, so it scales with the
  # SIZE OF THE CLI and not with what the test exercises. The uninstrumented
  # build makes the same point from the other side — 1.70 s and 1.77 s for the
  # same two commands against a 28 MB binary, a factor of 7.7 to 8.0 below the
  # instrumented figures.
  #
  # Second, the margin was never there to lose. 45 s bought about 5.6 s of
  # instrumented work, and one instrumented invocation already measured 12 s on
  # `main`, on a machine faster than any hosted runner. The ProjectWorkflow note
  # records that same suite costing 222 s locally against 517-740 s hosted, so
  # the hosted machine runs this workload 2.3 to 3.3 times slower: `main` sat at
  # roughly 28-40 s against a 45 s deadline. This branch adds capabilities to
  # the CLI, 16.8 percent of its instrumented size and 13.5 percent of its
  # per-invocation cost, and that consumed what was left. Run 34506763806 lost
  # FIFTEEN invocations to the deadline at once across all four suites — eleven
  # `project run`, four `project import-linear` — with ProjectLinearImport
  # losing its very first call in setUpClass and running zero tests. The same
  # four suites passed 518/518 in run 34471404322.
  #
  # So the deadline is SCALED BY THE MEASURED FACTOR rather than raised until
  # the suite goes green: 45 x 7.96 = 358, declared here as 360. Against a
  # hosted instrumented invocation of 32-45 s that is eight to eleven times the
  # work, comparable headroom to what 45 s gave the debug build, and it stays
  # below every module TIMEOUT set below, so it still fires first and still
  # names the command. The uninstrumented default is untouched at 45.
  #
  # This is a wall-clock guard, NOT an error budget on a computed quantity. No
  # numerical tolerance moves, no comparison loosens, no required outcome is
  # relaxed and no test changes what it asserts — the same reasoning the
  # ProjectWorkflow note sets out for its own figure, applied one level down. If
  # the CLI keeps growing the factor is re-measured with the command above; it
  # is not nudged.
  if(GALATA_ENABLE_ASAN OR GALATA_ENABLE_UBSAN)
    set(galata_project_timeout_s 360)
  else()
    set(galata_project_timeout_s 45)
  endif()

  add_test(NAME ProjectWorkflow
    COMMAND "${CMAKE_COMMAND}" -E env "GALATA_PROJECT_CLI=$<TARGET_FILE:galata_cli>"
      "GALATA_PROJECT_TIMEOUT_S=${galata_project_timeout_s}"
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
  # A 1.2 percent margin is not a margin. Six hosted sanitizer observations of
  # this one test are known, and they are recorded here in full rather than as
  # the subset that makes the margin look tidiest:
  #
  #   517.44 s  passed   run 34294391692
  #   532.09 s  passed   run 34290270469
  #   592.91 s  passed   `main` at f19c562, under the old 600 s limit
  #   600.04 s  TIMEOUT  run 34275003315 — the only failure of the six
  #   632.66 s  passed   run 34295580969, under this 1500 s limit
  #   740.55 s  passed   run 34283329227, under this 1500 s limit
  #
  # ELAPSED TIME IS NOT A FAILURE, and the two must not be counted together.
  # THREE of the six exceed 600 s, but exactly ONE of them was a timeout: the
  # 600.04 s run, which is the observation that prompted this change. The 632.66
  # and 740.55 s runs are elapsed-time observations from jobs that PASSED under
  # the raised limit — evidence about cost, not about correctness, and neither
  # is a gate that let anything through. A reader who counts three exceedances
  # as three failures is reading a variance record as a defect record.
  #
  # The spread is 517.44 to 740.55 s — 43 percent on a machine nobody here
  # controls. 1500 s is about twice the widest observation and nearly three
  # times the narrowest, which absorbs that variance and further CLI growth
  # while still catching a genuine hang inside half an hour.
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
      "GALATA_PROJECT_TIMEOUT_S=${galata_project_timeout_s}"
      "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/scripts/test_project_linear_import.py")
  set_tests_properties(ProjectLinearImport PROPERTIES LABELS "integration" TIMEOUT 600)
  add_test(NAME ProjectRecovery
    COMMAND "${CMAKE_COMMAND}" -E env "GALATA_PROJECT_CLI=$<TARGET_FILE:galata_cli>"
      "GALATA_PROJECT_TIMEOUT_S=${galata_project_timeout_s}"
      "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/scripts/test_project_recovery.py")
  set_tests_properties(ProjectRecovery PROPERTIES LABELS "integration" TIMEOUT 600)
  add_test(NAME ProjectRoutes
    COMMAND "${CMAKE_COMMAND}" -E env "GALATA_PROJECT_CLI=$<TARGET_FILE:galata_cli>"
      "GALATA_PROJECT_TIMEOUT_S=${galata_project_timeout_s}"
      "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/scripts/test_project_routes.py")
  set_tests_properties(ProjectRoutes PROPERTIES LABELS "integration" TIMEOUT 600)
  add_test(NAME DesktopPackageContract
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/scripts/test_desktop_packaging.py")
  set_tests_properties(DesktopPackageContract PROPERTIES LABELS "integration" TIMEOUT 60)
endif()
