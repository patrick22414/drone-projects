cmake_minimum_required(VERSION 3.10)
project(catch)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(MAVSDK REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

option(WITH_DRONE "Build with this if a drone or simulator is not available" ON)

if (WITH_DRONE)
    message(STATUS "Building WITH DRONE")
    add_definitions(-DUSE_DRONE)
else ()
    message(STATUS "Building WITHOUT DRONE")
endif ()

add_executable(catch src/main.cpp src/transforms.cpp src/common.cpp)

target_link_libraries(catch
        stdc++fs
        ${OpenCV_LIBS}
        MAVSDK::mavsdk_action
        MAVSDK::mavsdk_offboard
        MAVSDK::mavsdk_telemetry
        MAVSDK::mavsdk)