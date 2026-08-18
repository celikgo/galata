# SPDX-License-Identifier: Apache-2.0
#
# Eigen, with its headers marked SYSTEM.
#
# WHY THIS EXISTS. galata compiles with a deliberately aggressive warning set
# and -Werror. Those warnings are about galata's code and must not be applied to
# a third-party header-only library, which is compiled into every translation
# unit that includes it.
#
# CMake treats an IMPORTED target's interface include directories as SYSTEM
# automatically. vcpkg's Eigen3Config defines Eigen3::Eigen as a plain INTERFACE
# library rather than an IMPORTED one, so that does not happen, and Eigen's
# headers are compiled under galata's warning policy.
#
# The symptom, on GCC 13 at -O2 with -Wnull-dereference:
#
#   Eigen/src/Core/CoreEvaluators.h:911:56: error: null pointer dereference
#         [-Werror=null-dereference]
#     911 |     return m_data[col * colStride() + row * rowStride()];
#
# reached through an "inlined from" chain rooted in
# Eigen/src/Core/SolveTriangular.h, from an ldlt().solve() on a fixed-size 3x3.
# It is a false positive — GCC loses track of the fixed-size storage through
# several layers of expression-template inlining — and it appears only on GCC,
# only at -O2, and only on Linux. Clang, AppleClang and MSVC all build it clean.
#
# The wrong fixes, and why:
#   * Dropping -Wnull-dereference project-wide: it is a useful warning about
#     galata's own code, and one compiler's inlining artefact is no reason to
#     stop hearing it.
#   * Replacing ldlt().solve() with an explicit inverse: that changes a
#     numerical decision to work around a compiler bug, which is exactly
#     backwards. The comment in rigid_body.cpp explains why the solve is
#     preferred, and that reasoning does not depend on GCC's diagnostics.
#   * A #pragma GCC diagnostic around each call site: correct but viral — every
#     future Eigen call would need one.
#
# Marking the headers SYSTEM addresses the actual cause: they are somebody
# else's code, and our warning policy has no business applying to them.
#
# Written with the pre-3.25 spelling (INTERFACE_SYSTEM_INCLUDE_DIRECTORIES)
# rather than the 3.25+ SYSTEM target property, because CMakeLists.txt requires
# only 3.24.

include_guard(GLOBAL)

macro(galata_find_eigen)
  find_package(Eigen3 3.4 CONFIG REQUIRED)

  get_target_property(_galata_eigen_dirs Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  if(_galata_eigen_dirs)
    set_target_properties(Eigen3::Eigen PROPERTIES
      INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_galata_eigen_dirs}")
  endif()
  unset(_galata_eigen_dirs)
endmacro()
