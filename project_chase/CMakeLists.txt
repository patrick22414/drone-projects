cmake_minimum_required(VERSION 3.10)
project(project_chase)

set(CMAKE_CXX_STANDARD 17)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(chase src/main.cpp src/_common.h src/Chase2D.cpp src/Chase2D.h)

target_link_libraries(chase
        Eigen3::Eigen
        MAVSDK::mavsdk
        MAVSDK::mavsdk_action
        MAVSDK::mavsdk_offboard
        MAVSDK::mavsdk_telemetry
        ${OpenCV_LIBS}
        pthread)