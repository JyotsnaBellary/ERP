cmake_minimum_required(VERSION 3.10)
project(GraphReader)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Tell compiler where to find header files
include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/utils
    ${PROJECT_SOURCE_DIR}/src/data_structures
    ${PROJECT_SOURCE_DIR}/src/algorithms
)

# Collect all source files recursively
file(GLOB_RECURSE SOURCES
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

# Create executable from sources
add_executable(graph_reader ${SOURCES})
