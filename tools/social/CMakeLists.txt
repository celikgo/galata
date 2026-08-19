# SPDX-License-Identifier: Apache-2.0
#
# Emits the pole map that docs/assets/social-preview.png is drawn from, so the
# picture is a run rather than a transcription. See tools/social/main.cpp.

galata_find_eigen()

add_executable(galata-modal-map
  main.cpp
)

target_link_libraries(galata-modal-map
  PRIVATE
    galata::core
    galata::model
    galata::trim
    galata::linearize
    galata::analyze
    galata::warnings
)

target_compile_definitions(galata-modal-map
  PRIVATE GALATA_SOCIAL_MODEL="${CMAKE_SOURCE_DIR}/models/nt33a/nt33a-fc1.yaml"
)

set_target_properties(galata-modal-map PROPERTIES FOLDER "tools")
