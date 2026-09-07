# SPDX-License-Identifier: Apache-2.0
if(NOT APPLE)
  message(FATAL_ERROR "The native desktop feasibility preview currently supports macOS only")
endif()
enable_language(OBJCXX)
find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
# Match the deployment requirement of this build. This is not a declaration of
# minimum-OS product support for the feasibility preview.
if(CMAKE_OSX_DEPLOYMENT_TARGET)
  set(GALATA_DESKTOP_MINIMUM_SYSTEM_VERSION "${CMAKE_OSX_DEPLOYMENT_TARGET}")
else()
  execute_process(COMMAND /usr/bin/sw_vers -productVersion
    RESULT_VARIABLE _galata_macos_version_result
    OUTPUT_VARIABLE GALATA_DESKTOP_MINIMUM_SYSTEM_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT _galata_macos_version_result EQUAL 0)
    message(FATAL_ERROR "Cannot determine the desktop bundle's macOS deployment requirement")
  endif()
endif()
if(NOT GALATA_DESKTOP_MINIMUM_SYSTEM_VERSION MATCHES "^[0-9]+([.][0-9]+)*$")
  message(FATAL_ERROR "The desktop bundle's macOS deployment requirement must be a version number")
endif()
add_executable(galata_desktop MACOSX_BUNDLE main.mm accessible_views.mm diagram_routing.mm
  connection_editing.mm diagram_editing.mm)
set_target_properties(galata_desktop PROPERTIES
  OUTPUT_NAME "Galata Preview"
  MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in"
  MACOSX_BUNDLE_BUNDLE_NAME "Galata Preview"
  MACOSX_BUNDLE_GUI_IDENTIFIER "org.galata.desktop.preview"
  MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
  MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
  OBJCXX_STANDARD 20
  OBJCXX_STANDARD_REQUIRED YES
  FOLDER "tools")
target_compile_options(galata_desktop PRIVATE -fobjc-arc -Wall -Wextra -Wpedantic)
if(GALATA_WERROR)
  target_compile_options(galata_desktop PRIVATE -Werror)
endif()
target_link_libraries(galata_desktop PRIVATE "-framework AppKit" "-framework Foundation"
  "-framework QuartzCore")
# The UI always launches the same CLI implementation used by batch studies.
add_dependencies(galata_desktop galata_cli)
set_property(TARGET galata_desktop APPEND PROPERTY LINK_DEPENDS "$<TARGET_FILE:galata_cli>")
set_property(TARGET galata_desktop APPEND PROPERTY LINK_DEPENDS
  "${PROJECT_SOURCE_DIR}/scripts/package-desktop.py"
  "${PROJECT_SOURCE_DIR}/scripts/check-desktop-package.py")
add_custom_command(TARGET galata_desktop POST_BUILD
  COMMAND "${CMAKE_COMMAND}" -E copy_if_different "$<TARGET_FILE:galata_cli>"
    "$<TARGET_BUNDLE_DIR:galata_desktop>/Contents/MacOS/galata"
  COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/scripts/package-desktop.py"
    "${PROJECT_BINARY_DIR}" --stamp-app "$<TARGET_BUNDLE_DIR:galata_desktop>"
    --stamp-output "${CMAKE_CURRENT_BINARY_DIR}/galata-desktop-build.json"
  VERBATIM)
