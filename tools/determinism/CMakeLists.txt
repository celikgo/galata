# SPDX-License-Identifier: Apache-2.0

# The battery, shared with the V&V report generator so that "how many values
# does the determinism gate cover" has one answer rather than two.
add_library(galata_determinism_fingerprint
  fingerprint.cpp
)
add_library(galata::determinism_fingerprint ALIAS galata_determinism_fingerprint)

target_include_directories(galata_determinism_fingerprint
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(galata_determinism_fingerprint
  PUBLIC
    galata::core
    galata::numerics
    galata::sim
    galata::analyze
    galata::model
    galata::trim
    galata::linearize
  PRIVATE
    galata::warnings
)

set_target_properties(galata_determinism_fingerprint PROPERTIES FOLDER "tools")

add_executable(galata-determinism
  main.cpp
)

# The shipped model the battery trims. Compiled in rather than passed as an
# argument, so a fingerprint cannot silently be taken of a different aircraft.
target_compile_definitions(galata-determinism
  PRIVATE GALATA_DETERMINISM_MODEL="${CMAKE_SOURCE_DIR}/models/nt33a/nt33a-fc1.yaml"
)

target_link_libraries(galata-determinism
  PRIVATE
    galata::determinism_fingerprint
    galata::core
    galata::warnings
)

set_target_properties(galata-determinism PROPERTIES FOLDER "tools")
