# SPDX-License-Identifier: Apache-2.0
#
# galata_identify: fitting measured records to declared model forms.

galata_find_eigen()

add_library(galata_identify
  static_fit.cpp
  greybox.cpp
  validate.cpp
)
add_library(galata::identify ALIAS galata_identify)

target_link_libraries(galata_identify
  PUBLIC
    galata::public_headers
    galata::data
    galata::model
    galata::numerics
    Eigen3::Eigen
  PRIVATE
    galata::warnings
)

set_target_properties(galata_identify PROPERTIES FOLDER "libraries")
