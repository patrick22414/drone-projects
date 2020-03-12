cmake_minimum_required(VERSION 3.1.0)
project(project_chase CXX)

set(CMAKE_CXX_STANDARD 17)

message("")
message("External library fmt:")
add_subdirectory(external/fmt)

message("")
message("Project chase:")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)

option(WITH_DRONE "Whether drone connection is available" ON)

if (WITH_DRONE)
    message("Building WITH drone connection")
    add_definitions(-DWITH_DRONE)
else ()
    message("Building WITHOUT drone connection")
endif ()

add_executable(chase
        src/main.cpp
        src/chase2d.cpp)

include_directories(
        external/include
        external/fmt/include
        ${OpenCV_INCLUDE_DIRS})

target_link_libraries(chase
        Eigen3::Eigen
        fmt::fmt
        MAVSDK::mavsdk
        MAVSDK::mavsdk_action
        MAVSDK::mavsdk_offboard
        MAVSDK::mavsdk_telemetry
        ${OpenCV_LIBS})

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message("Add extra link libraries for ${CMAKE_CXX_COMPILER_ID}")
    target_link_libraries(chase
            stdc++fs)
endif ()