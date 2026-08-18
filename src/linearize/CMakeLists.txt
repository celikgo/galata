# SPDX-License-Identifier: Apache-2.0
#
# galata_linearize: numerical Jacobians assembled into state-space models.

galata_find_eigen()

add_library(galata_linearize
  finite_difference.cpp
)
add_library(galata::linearize ALIAS galata_linearize)

target_link_libraries(galata_linearize
  PUBLIC
    galata::public_headers
    galata::core
    galata::model
    galata::numerics
    galata::trim
    Eigen3::Eigen
  PRIVATE
    galata::warnings
)

set_target_properties(galata_linearize PROPERTIES FOLDER "libraries")
