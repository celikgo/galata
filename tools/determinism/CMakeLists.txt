# SPDX-License-Identifier: Apache-2.0

add_executable(galata-determinism
  main.cpp
)

target_link_libraries(galata-determinism
  PRIVATE
    galata::core
    galata::numerics
    galata::sim
    galata::analyze
    galata::warnings
)

set_target_properties(galata-determinism PROPERTIES FOLDER "tools")
