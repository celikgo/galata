# SPDX-License-Identifier: Apache-2.0
# Headless presentation-geometry regressions for the optional native preview.
# The application enables Objective-C++ in a sibling directory; CMake's language
# rule variables must also be initialized in this target's directory scope.
enable_language(OBJCXX)
add_executable(galata_desktop_tests
  test_diagram_routing.mm
  test_connection_editing.mm
  test_diagram_editing.mm
  "${CMAKE_SOURCE_DIR}/src/desktop/diagram_routing.mm"
  "${CMAKE_SOURCE_DIR}/src/desktop/connection_editing.mm"
  "${CMAKE_SOURCE_DIR}/src/desktop/diagram_editing.mm"
)
set_target_properties(galata_desktop_tests PROPERTIES
  OBJCXX_STANDARD 20
  OBJCXX_STANDARD_REQUIRED YES
  FOLDER "tests"
)
target_include_directories(galata_desktop_tests PRIVATE "${CMAKE_SOURCE_DIR}/src/desktop")
target_compile_options(galata_desktop_tests PRIVATE -fobjc-arc -Wall -Wextra -Wpedantic)
if(GALATA_WERROR)
  target_compile_options(galata_desktop_tests PRIVATE -Werror)
endif()
target_link_libraries(galata_desktop_tests PRIVATE
  GTest::gtest_main "-framework AppKit" "-framework Foundation"
)
gtest_discover_tests(galata_desktop_tests
  PROPERTIES LABELS "unit" TIMEOUT 30
  DISCOVERY_TIMEOUT 30
)
