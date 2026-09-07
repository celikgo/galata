# SPDX-License-Identifier: Apache-2.0
find_package(yaml-cpp CONFIG REQUIRED)

add_executable(galata_cli
  main.cpp
  project.cpp
)

set_target_properties(galata_cli PROPERTIES
  OUTPUT_NAME galata
  FOLDER "tools"
)

target_link_libraries(galata_cli
  PRIVATE
    galata::core
    galata::pipeline
    yaml-cpp::yaml-cpp
    galata::warnings
)

if(APPLE)
  # The relocated desktop worker must verify on Intel as well as Apple Silicon;
  # not every linker adds an ad-hoc signature by default. This local integrity
  # seal is not a Developer ID signature or notarization.
  add_custom_command(TARGET galata_cli POST_BUILD
    COMMAND /usr/bin/codesign --force --sign - --timestamp=none "$<TARGET_FILE:galata_cli>"
    VERBATIM)
endif()
