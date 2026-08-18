# SPDX-License-Identifier: Apache-2.0
#
# galata_sim: rigid-body dynamics and the nonlinear simulation loop.

find_package(Eigen3 3.4 CONFIG REQUIRED)

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
