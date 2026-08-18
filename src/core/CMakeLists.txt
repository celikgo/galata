# SPDX-License-Identifier: Apache-2.0
#
# galata_core: frames, quaternions, units, state, atmosphere, equations of
# motion. Everything numerical depends on this; it depends on nothing but Eigen
# and the standard library.

# Version pinned in vcpkg.json's `overrides`, not floated against the registry
# baseline: Eigen's version is part of what makes a numerical result
# reproducible (ADR-0004). EXACT rather than a minimum, so that a registry
# roll to Eigen 5 is a deliberate edit with a determinism re-baseline rather
# than something that happens on a Tuesday. The floor here rejects a major
# bump; the manifest override fixes the exact build.
find_package(Eigen3 3.4 CONFIG REQUIRED)

add_library(galata_core
  version.cpp
  quaternion.cpp
  frames.cpp
  state.cpp
  atmosphere.cpp
)
add_library(galata::core ALIAS galata_core)

target_link_libraries(galata_core
  PUBLIC
    galata::public_headers
    # PUBLIC because galata/core/quaternion.hpp and state.hpp expose Eigen
    # types in their signatures. Eigen3::Eigen is an imported target, so its
    # include directory is treated as SYSTEM and Eigen's own warnings do not
    # reach -Werror.
    Eigen3::Eigen
  PRIVATE
    galata::warnings
)

set_target_properties(galata_core PROPERTIES
  OUTPUT_NAME galata_core
  FOLDER "libraries"
)
