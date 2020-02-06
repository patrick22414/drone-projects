cmake_minimum_required(VERSION 3.10)
project(_catch)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(MAVSDK REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

#add_compile_definitions(USE_DRONE_CONTROL)

add_executable(_catch src/main.cpp include/internal.h)

target_link_libraries(_catch
        ${OpenCV_LIBS}
        MAVSDK::mavsdk_action
        MAVSDK::mavsdk_offboard
        MAVSDK::mavsdk_telemetry
        MAVSDK::mavsdk)