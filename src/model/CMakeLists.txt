# SPDX-License-Identifier: Apache-2.0
#
# galata_model: aircraft models, aerodynamic coefficient sets, mass properties
# and the linear systems they produce.

galata_find_eigen()
find_package(yaml-cpp CONFIG REQUIRED)

add_library(galata_model
  linear_system.cpp
)
add_library(galata::model ALIAS galata_model)

target_link_libraries(galata_model
  PUBLIC
    galata::public_headers
    Eigen3::Eigen
  PRIVATE
    galata::core
    galata::warnings
    yaml-cpp::yaml-cpp
)

set_target_properties(galata_model PROPERTIES FOLDER "libraries")
