# SPDX-License-Identifier: Apache-2.0
add_library(galata_onboard controller_plugin.cpp deployment.cpp runtime.cpp target_evidence.cpp)
add_library(galata::onboard ALIAS galata_onboard)

target_link_libraries(galata_onboard
  PUBLIC
    galata::public_headers
    galata::hardware
  PRIVATE
    galata::core
    galata::warnings
    ${CMAKE_DL_LIBS}
)

set_target_properties(galata_onboard PROPERTIES FOLDER "libraries")
