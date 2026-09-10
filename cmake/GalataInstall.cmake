# SPDX-License-Identifier: Apache-2.0
# Include after every library has been declared. The C++ package is pre-1.0;
# matching version does not promise a frozen C++ ABI (ADR-0001).
include_guard(GLOBAL)
include(CMakePackageConfigHelpers)

set(_galata_install_targets galata_public_headers galata_warnings)
foreach(_part core data numerics sim simulation analyze model trim linearize pipeline synth modeling)
  if(TARGET galata_${_part})
    set_target_properties(galata_${_part} PROPERTIES EXPORT_NAME ${_part})
    target_compile_features(galata_${_part} PUBLIC cxx_std_20)
    list(APPEND _galata_install_targets galata_${_part})
  endif()
endforeach()
set_target_properties(galata_public_headers PROPERTIES EXPORT_NAME public_headers)
set_target_properties(galata_warnings PROPERTIES EXPORT_NAME warnings)
# A static library's PRIVATE dependencies still appear in its link interface.
# Export the warnings target, but keep our warning policy out of consumer builds.
foreach(_property INTERFACE_COMPILE_OPTIONS INTERFACE_COMPILE_DEFINITIONS)
  get_target_property(_value galata_warnings ${_property})
  if(_value)
    set_property(TARGET galata_warnings PROPERTY ${_property} "$<BUILD_INTERFACE:${_value}>")
  endif()
endforeach()
install(TARGETS ${_galata_install_targets} EXPORT GalataTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
if(TARGET galata_cli)
  install(TARGETS galata_cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/galata"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(FILES "${PROJECT_BINARY_DIR}/generated/include/galata/build_config.hpp"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/galata")
set(_galata_package_dir "${CMAKE_INSTALL_LIBDIR}/cmake/Galata")
configure_package_config_file("${CMAKE_CURRENT_LIST_DIR}/GalataConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/GalataConfig.cmake" INSTALL_DESTINATION "${_galata_package_dir}")
write_basic_package_version_file("${PROJECT_BINARY_DIR}/GalataConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}" COMPATIBILITY ExactVersion)
install(EXPORT GalataTargets NAMESPACE galata:: DESTINATION "${_galata_package_dir}")
install(FILES "${PROJECT_BINARY_DIR}/GalataConfig.cmake"
  "${PROJECT_BINARY_DIR}/GalataConfigVersion.cmake" DESTINATION "${_galata_package_dir}")
install(FILES "${PROJECT_SOURCE_DIR}/LICENSE" "${PROJECT_SOURCE_DIR}/NOTICE"
  "${PROJECT_SOURCE_DIR}/THIRD_PARTY_LICENSES.md" DESTINATION "${CMAKE_INSTALL_DATADIR}/galata")
install(DIRECTORY "${PROJECT_SOURCE_DIR}/third_party/licenses"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/galata/third_party"
  PATTERN "eigen3.txt" EXCLUDE PATTERN "yaml-cpp.txt" EXCLUDE)
# These override the source reference copies with the notices belonging to the
# dependencies resolved by this build. Other package managers can supply paths.
set(GALATA_EIGEN_LICENSE_FILE "${Eigen3_DIR}/copyright" CACHE FILEPATH
  "Copyright/license notice supplied with the resolved Eigen package")
set(GALATA_YAML_CPP_LICENSE_FILE "${yaml-cpp_DIR}/copyright" CACHE FILEPATH
  "Copyright/license notice supplied with the resolved yaml-cpp package")
foreach(_dependency EIGEN YAML_CPP)
  if(NOT EXISTS "${GALATA_${_dependency}_LICENSE_FILE}")
    message(FATAL_ERROR "Set GALATA_${_dependency}_LICENSE_FILE to the resolved dependency's notice file")
  endif()
endforeach()
install(FILES "${GALATA_EIGEN_LICENSE_FILE}"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/galata/third_party/licenses" RENAME eigen3.txt)
install(FILES "${GALATA_YAML_CPP_LICENSE_FILE}"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/galata/third_party/licenses" RENAME yaml-cpp.txt)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/examples" "${PROJECT_SOURCE_DIR}/models"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/galata" FILES_MATCHING
  PATTERN "*.yaml" PATTERN "README.md" PATTERN "PROVENANCE.md")
install(FILES "${PROJECT_SOURCE_DIR}/vcpkg.json" DESTINATION "${CMAKE_INSTALL_DATADIR}/galata")
install(FILES "${PROJECT_SOURCE_DIR}/cmake/GalataInstall.cmake"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/galata/cmake")
install(DIRECTORY "${PROJECT_SOURCE_DIR}/tests/validation/reference"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/galata/tests/validation")
install(DIRECTORY "${PROJECT_SOURCE_DIR}/docs/adr"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/galata/docs")

if(TARGET galata_desktop)
  install(TARGETS galata_desktop BUNDLE DESTINATION ".")
  install(PROGRAMS "$<TARGET_FILE:galata_cli>"
    DESTINATION "Galata Preview.app/Contents/MacOS")
  install(FILES "${PROJECT_SOURCE_DIR}/LICENSE" "${PROJECT_SOURCE_DIR}/NOTICE"
    "${PROJECT_SOURCE_DIR}/THIRD_PARTY_LICENSES.md"
    DESTINATION "Galata Preview.app/Contents/Resources")
  install(FILES "${GALATA_EIGEN_LICENSE_FILE}" DESTINATION
    "Galata Preview.app/Contents/Resources/licenses" RENAME eigen3.txt)
  install(FILES "${GALATA_YAML_CPP_LICENSE_FILE}" DESTINATION
    "Galata Preview.app/Contents/Resources/licenses" RENAME yaml-cpp.txt)
endif()
