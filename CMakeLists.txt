cmake_minimum_required(VERSION 3.10)
project(project)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/external 
)

add_library(mesh SHARED
    src/mesh.cpp
    src/half_edge.cpp
)

add_executable(simplify simplify.cpp)
target_link_libraries(simplify mesh)
