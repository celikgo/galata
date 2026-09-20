# SPDX-License-Identifier: Apache-2.0
add_library(galata_qualification dossier.cpp)
add_library(galata::qualification ALIAS galata_qualification)

target_link_libraries(galata_qualification
  PUBLIC
    galata::public_headers
  PRIVATE
    galata::core
    galata::warnings
)

set_target_properties(galata_qualification PROPERTIES FOLDER "libraries")
