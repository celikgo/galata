# SPDX-License-Identifier: Apache-2.0

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
    galata::core
    galata::numerics
    galata::sim
    galata::analyze
    galata::model
    galata::trim
    galata::linearize
    galata::warnings
)

set_target_properties(galata-determinism PROPERTIES FOLDER "tools")
