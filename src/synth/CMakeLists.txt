# SPDX-License-Identifier: Apache-2.0
galata_find_eigen()
add_library(galata_synth control.cpp discrete_control.cpp)
add_library(galata::synth ALIAS galata_synth)
target_link_libraries(galata_synth PUBLIC galata::public_headers galata::model galata::numerics Eigen3::Eigen
  PRIVATE galata::warnings)
set_target_properties(galata_synth PROPERTIES FOLDER "libraries")
