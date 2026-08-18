# SPDX-License-Identifier: Apache-2.0
#
# galata_core: frames, quaternions, units, state, atmosphere, equations of
# motion. Everything numerical depends on this and it depends on nothing but
# Eigen and the standard library.

add_library(galata_core
  version.cpp
)
add_library(galata::core ALIAS galata_core)

target_link_libraries(galata_core
  PUBLIC
    galata::public_headers
  PRIVATE
    galata::warnings
)

set_target_properties(galata_core PROPERTIES
  OUTPUT_NAME galata_core
  FOLDER "libraries"
)
