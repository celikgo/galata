# SPDX-License-Identifier: Apache-2.0
add_subdirectory(core)
add_subdirectory(numerics)
add_subdirectory(sim)
add_subdirectory(analyze)
add_subdirectory(model)
add_subdirectory(pipeline)

if(GALATA_BUILD_CLI)
  add_subdirectory(cli)
endif()
