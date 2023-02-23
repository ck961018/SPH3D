cmake_minimum_required(VERSION 3.10)
project(SPH3D)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=-*,readability-*)

find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/3rdparty/flatbuffers)
include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB_RECURSE SRC_LIST ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(SPH3D ${SRC_LIST})

target_link_libraries(SPH3D PRIVATE glad::glad) 
target_link_libraries(SPH3D PRIVATE glfw)
target_link_libraries(SPH3D PRIVATE glm::glm)
target_link_libraries(SPH3D PRIVATE TBB::tbb TBB::tbbmalloc TBB::tbbmalloc_proxy)