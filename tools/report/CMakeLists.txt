# SPDX-License-Identifier: Apache-2.0
#
# Emits the run that docs/reports/ is drawn from, so the report page is a run
# rather than a transcription. See tools/report/main.cpp.

galata_find_eigen()

add_executable(galata-report-data
  main.cpp
)

target_link_libraries(galata-report-data
  PRIVATE
    galata::core
    galata::model
    galata::trim
    galata::linearize
    galata::analyze
    galata::validation_reference
    galata::warnings
)

target_compile_definitions(galata-report-data
  PRIVATE
    GALATA_CASE_MODEL="${CMAKE_SOURCE_DIR}/models/nt33a/nt33a-fc1.yaml"
    GALATA_CASE_REFERENCE_DIR="${CMAKE_SOURCE_DIR}/tests/validation/reference"
)

set_target_properties(galata-report-data PROPERTIES FOLDER "tools")
