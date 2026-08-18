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
)
add_library(galata::pipeline ALIAS galata_pipeline)

target_link_libraries(galata_pipeline
  PUBLIC
    galata::public_headers
    galata::model
    galata::analyze
    galata::trim
    galata::linearize
    Eigen3::Eigen
  PRIVATE
    galata::core
    galata::warnings
    yaml-cpp::yaml-cpp
)

set_target_properties(galata_pipeline PROPERTIES FOLDER "libraries")
