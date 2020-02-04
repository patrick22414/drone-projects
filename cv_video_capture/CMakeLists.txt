cmake_minimum_required(VERSION 3.10)
project(rpi_video_capture)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(rpi_video_capture main.cpp)
target_link_libraries(rpi_video_capture
        ${OpenCV_LIBS})