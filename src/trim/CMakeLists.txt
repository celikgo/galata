# SPDX-License-Identifier: Apache-2.0
#
# galata_trim: trim problem formulations and their solvers.

galata_find_eigen()

add_library(galata_trim
  level.cpp
)
add_library(galata::trim ALIAS galata_trim)

target_link_libraries(galata_trim
  PUBLIC
    galata::public_headers
    galata::core
    galata::model
    Eigen3::Eigen
  PRIVATE
    galata::numerics
    galata::warnings
)

set_target_properties(galata_trim PROPERTIES FOLDER "libraries")
