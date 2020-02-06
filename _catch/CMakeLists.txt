cmake_minimum_required(VERSION 3.10)
project(_catch)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(MAVSDK REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_executable(_catch src/main.cpp)

target_link_libraries(_catch
        ${OpenCV_LIBS}
        MAVSDK::mavsdk_telemetry
        MAVSDK::mavsdk_action
        MAVSDK::mavsdk)