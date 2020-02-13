cmake_minimum_required(VERSION 3.10)
project(cv_tracking)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(cv_tracking main.cpp)

target_link_libraries(cv_tracking
        ${OpenCV_LIBS})