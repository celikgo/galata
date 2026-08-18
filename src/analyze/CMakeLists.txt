# SPDX-License-Identifier: Apache-2.0
#
# galata_analyze: modal decomposition, frequency response, margins, handling
# qualities. Consumes a linear system; knows nothing about how it was produced.

galata_find_eigen()

add_library(galata_analyze
  modes.cpp
)
add_library(galata::analyze ALIAS galata_analyze)

target_link_libraries(galata_analyze
  PUBLIC
    galata::public_headers
    Eigen3::Eigen
  PRIVATE
    galata::core
    galata::warnings
)

set_target_properties(galata_analyze PROPERTIES FOLDER "libraries")
