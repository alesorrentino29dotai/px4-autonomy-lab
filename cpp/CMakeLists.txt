cmake_minimum_required(VERSION 3.16)
project(px4_autonomy_lab_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# libmavsdk extracted locally by get_mavsdk.sh (no system install / no sudo)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mavsdk/usr")

find_package(MAVSDK REQUIRED)

add_executable(m6_takeoff_land m6_takeoff_land.cpp)
target_link_libraries(m6_takeoff_land MAVSDK::mavsdk)

# find libmavsdk.so at runtime without LD_LIBRARY_PATH
set_target_properties(m6_takeoff_land PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mavsdk/usr/lib")
