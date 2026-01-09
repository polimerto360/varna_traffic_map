# CMakeList.txt : CMake project for traffic_map, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("traffic_map")

# Add source to this project's executable.
add_executable (traffic_map "traffic_map.cpp" "traffic_map.h")

# TODO: Add tests and install targets if needed.

# Add Geodesk library
include(FetchContent)
FetchContent_Declare(geodesk GIT_REPOSITORY 
    https://github.com/clarisma/libgeodesk.git
    GIT_TAG main)
FetchContent_MakeAvailable(geodesk)

target_link_libraries(traffic_map geodesk)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
