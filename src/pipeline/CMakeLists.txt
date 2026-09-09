# SPDX-License-Identifier: Apache-2.0
#
# galata_pipeline: the YAML DAG, the capability registry, and the capabilities
# this build provides.

galata_find_eigen()
find_package(yaml-cpp CONFIG REQUIRED)

add_library(galata_pipeline
  value.cpp
  registry.cpp
  pipeline.cpp
  capabilities.cpp
  design_capabilities.cpp
  model_capabilities.cpp
  quadrotor_capabilities.cpp
  linear_graph_capability.cpp
  files.cpp
  sha256.cpp
  provenance.cpp
  runtime_identity.cpp
  html_report.cpp
  svg_charts.cpp
)
add_library(galata::pipeline ALIAS galata_pipeline)

target_link_libraries(galata_pipeline
  PUBLIC
    galata::public_headers
    galata::model
    galata::modeling
    galata::analyze
    galata::trim
    galata::linearize
    galata::synth
    galata::simulation
    Eigen3::Eigen
  PRIVATE
    galata::core
    galata::warnings
    ${CMAKE_DL_LIBS}
    yaml-cpp::yaml-cpp
)

set(_galata_provenance_header "${CMAKE_CURRENT_BINARY_DIR}/galata_pipeline_provenance.hpp")
add_custom_target(galata_provenance_stamp
  COMMAND "${CMAKE_COMMAND}"
    "-DGALATA_SOURCE_DIR=${CMAKE_SOURCE_DIR}"
    "-DGALATA_BINARY_DIR=${CMAKE_BINARY_DIR}"
    "-DGALATA_PROVENANCE_HEADER=${_galata_provenance_header}"
    "-DGALATA_EIGEN_VERSION=${EIGEN3_VERSION_STRING}"
    "-DGALATA_YAML_VERSION=${yaml-cpp_VERSION}"
    -P "${CMAKE_SOURCE_DIR}/cmake/GalataProvenance.cmake"
  BYPRODUCTS "${_galata_provenance_header}"
  VERBATIM)
add_dependencies(galata_pipeline galata_provenance_stamp)
target_include_directories(galata_pipeline PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")

set_target_properties(galata_pipeline PROPERTIES FOLDER "libraries")
