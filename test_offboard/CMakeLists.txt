cmake_minimum_required(VERSION 2.8.12)

project(offboard)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(MAVSDK REQUIRED)

option(USE_VELOCITY_CONTROL "Use set_velocity instead of set_position" ON)

if (USE_VELOCITY_CONTROL)
    message(STATUS "Using velocity control (set_velocity_ned)")
    add_definitions(-DUSE_VELOCITY_CONTROL)
else ()
    message(STATUS "Using position control (set_position_ned)")
endif ()

add_executable(offboard
        offboard.cpp)

target_link_libraries(offboard
        ${OpenCV_LIBS}
        MAVSDK::mavsdk_action
        MAVSDK::mavsdk_offboard
        MAVSDK::mavsdk_telemetry
        MAVSDK::mavsdk
        stdc++fs
        pthread)
