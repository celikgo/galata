# SPDX-License-Identifier: Apache-2.0
galata_find_eigen()
find_package(yaml-cpp CONFIG REQUIRED)
add_library(galata_modeling model.cpp model_io.cpp)
add_library(galata::modeling ALIAS galata_modeling)
target_link_libraries(galata_modeling
  PUBLIC galata::public_headers Eigen3::Eigen
  PRIVATE galata::core galata::numerics galata::warnings yaml-cpp::yaml-cpp)
set_target_properties(galata_modeling PROPERTIES FOLDER "libraries")
