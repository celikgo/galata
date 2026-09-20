# SPDX-License-Identifier: Apache-2.0
add_library(galata_hardware interface.cpp profile.cpp can_transport.cpp serial_transport.cpp
  udp_transport.cpp)
add_library(galata::hardware ALIAS galata_hardware)

target_link_libraries(galata_hardware
  PUBLIC
    galata::public_headers
  PRIVATE
    galata::warnings
)

set_target_properties(galata_hardware PROPERTIES FOLDER "libraries")
