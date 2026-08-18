# SPDX-License-Identifier: Apache-2.0

add_executable(galata_cli
  main.cpp
)

set_target_properties(galata_cli PROPERTIES
  OUTPUT_NAME galata
  FOLDER "tools"
)

target_link_libraries(galata_cli
  PRIVATE
    galata::core
    galata::pipeline
    galata::warnings
)
