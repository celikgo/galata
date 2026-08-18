# SPDX-License-Identifier: Apache-2.0
#
# galata_numerics: integrators, root-finders, Jacobians, eigenproblems.
# Depends on Eigen and galata_core, and on nothing that knows about aircraft.

galata_find_eigen()

add_library(galata_numerics
  integrator.cpp
)
add_library(galata::numerics ALIAS galata_numerics)

target_link_libraries(galata_numerics
  PUBLIC
    galata::public_headers
    Eigen3::Eigen
  PRIVATE
    galata::warnings
)

set_target_properties(galata_numerics PROPERTIES FOLDER "libraries")
