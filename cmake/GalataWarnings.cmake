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
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    -Wimplicit-fallthrough
    -Wmisleading-indentation
    -Wold-style-cast
  )
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(galata_warnings INTERFACE
      -Wduplicated-cond
      -Wduplicated-branches
      -Wlogical-op
      -Wuseless-cast
    )
  endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  target_compile_options(galata_warnings INTERFACE
    /W4
    /permissive-
    /w14242  # narrowing conversion
    /w14254  # signed/unsigned mismatch
    /w14263  # member function does not override
    /w14265  # class has virtual functions but non-virtual dtor
    /w14287  # unsigned/negative constant mismatch
    /w14296  # expression is always true/false
    /w14311  # pointer truncation
    /w14545  # comma operator with no effect
    /w14546  # call with missing argument list
    /w14547  # operator before comma has no effect
    /w14549  # operator before comma has no effect
    /w14555  # expression has no effect
    /w14619  # unknown #pragma warning number
    /w14640  # non-thread-safe local static construction
    /w14826  # sign-extending conversion
    /w14905  # wide string literal cast
    /w14906  # string literal cast
    /w14928  # illegal copy-initialization
  )
  target_compile_definitions(galata_warnings INTERFACE
    _CRT_SECURE_NO_WARNINGS
    NOMINMAX
    WIN32_LEAN_AND_MEAN
    _USE_MATH_DEFINES
  )
endif()

if(GALATA_WERROR)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(galata_warnings INTERFACE -Werror)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(galata_warnings INTERFACE /WX)
  endif()
endif()
