# SPDX-License-Identifier: Apache-2.0
#
# galata_data: measured records, and the readers that produce them.
#
# This is a FILE-FORMAT ADAPTER layer. Unit conversion is expected here and
# nowhere below it: a source file arrives in whatever units its author chose,
# and the one place it becomes SI is a declared scale and offset recorded on the
# channel it produced (ADR-0003).

add_library(galata_data
  record.cpp
  csv.cpp
)
add_library(galata::data ALIAS galata_data)

target_link_libraries(galata_data
  PUBLIC
    galata::public_headers
    galata::core
  PRIVATE
    galata::warnings
)

set_target_properties(galata_data PROPERTIES FOLDER "libraries")
