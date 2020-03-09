cmake_minimum_required(VERSION 3.10)
project(test_mosse_tracking)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(mosse_tracking main.cpp)

target_link_libraries(mosse_tracking
        ${OpenCV_LIBS})