cmake_minimum_required(VERSION 3.10)
project(ball_tracking)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ball_tracking main.cpp)
target_link_libraries(ball_tracking
        ${OpenCV_LIBS})