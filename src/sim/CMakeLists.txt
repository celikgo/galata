# SPDX-License-Identifier: Apache-2.0
#
# galata_sim: rigid-body dynamics and the nonlinear simulation loop.

galata_find_eigen()

add_library(galata_sim
  rigid_body.cpp
)
add_library(galata::sim ALIAS galata_sim)

target_link_libraries(galata_sim
  PUBLIC
    galata::public_headers
    galata::core
    Eigen3::Eigen
  PRIVATE
    galata::warnings
)

set_target_properties(galata_sim PROPERTIES FOLDER "libraries")

# The aircraft model depends on rigid-body primitives. Keep the simulation
# drivers in a separate target so model -> sim does not become a link cycle.
add_library(galata_simulation
  nonlinear.cpp
  linear.cpp
)
add_library(galata::simulation ALIAS galata_simulation)
target_link_libraries(galata_simulation
  PUBLIC
    galata::model
    galata::trim
    galata::linearize
    galata::numerics
  PRIVATE
    galata::warnings
)
set_target_properties(galata_simulation PROPERTIES FOLDER "libraries")
