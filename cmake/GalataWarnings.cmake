# SPDX-License-Identifier: Apache-2.0
#
# Compiler warning flags applied to every galata target.
# Targets opt in via `target_link_libraries(target PRIVATE galata::warnings)`.

include_guard(GLOBAL)

add_library(galata_warnings INTERFACE)
add_library(galata::warnings ALIAS galata_warnings)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(galata_warnings INTERFACE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wconversion
    -Wsign-conversion
    -Wdouble-promotion
    -Wformat=2
    -Wimplicit-fallthrough
    -Wmisleading-indentation
    -Wold-style-cast
  )
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # Clang's -Wnull-dereference is a front-end diagnostic about dereferences it
    # can prove are null. It is precise and it is kept.
    #
    # GCC's warning of the same name is a middle-end diagnostic produced after
    # inlining, and on this codebase it is unusable. At -O2 it fires inside
    # Eigen's expression templates AND inside libstdc++'s own <complex>:
    #
    #   Eigen/src/Core/CoreEvaluators.h:911: potential null pointer dereference
    #   .../include/c++/15/complex:1633: potential null pointer dereference
    #
    # Marking those headers SYSTEM does not silence it, because the diagnostic
    # arises from template instantiation inside galata's own translation units
    # and GCC does not attribute such warnings to the header they textually
    # occur in. A #pragma cannot reliably suppress a middle-end warning either.
    #
    # A warning that fires on the standard library's implementation of
    # std::complex tells us nothing about galata, so on GCC it is not enabled.
    # It stays on for Clang, where it behaves.
    target_compile_options(galata_warnings INTERFACE -Wnull-dereference)
  endif()

  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(galata_warnings INTERFACE
      -Wduplicated-cond
      -Wduplicated-branches
      -Wlogical-op
      -Wuseless-cast
    )
  endif()
endif()

# A compiler that is none of those gets no warning flags at all, rather than a
# set guessed at its spelling. galata is built with GCC and Clang on Linux and
# with AppleClang on macOS, and those are the only warning sets any job holds to
# -Werror. The MSVC set that stood here went with Windows support: an unbuilt
# warning set is a claim about code nobody compiles.

if(GALATA_WERROR AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(galata_warnings INTERFACE -Werror)
endif()
