cmake_minimum_required(VERSION 3.0)
project(project_chase)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

option(WITH_DRONE "Whether drone connection is available" ON)

if (WITH_DRONE)
    message("Building WITH drone connection")
    add_definitions(-DWITH_DRONE)
else ()
    message("Building WITHOUT drone connection")
endif ()

add_executable(chase
        src/main.cpp
        src/Chase2D.cpp)

target_link_libraries(chase
        Eigen3::Eigen
        MAVSDK::mavsdk
        MAVSDK::mavsdk_action
        MAVSDK::mavsdk_offboard
        MAVSDK::mavsdk_telemetry
        ${OpenCV_LIBS}
        pthread)