cmake_minimum_required(VERSION 3.10)
project(diff_tracking)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package (Eigen3 REQUIRED NO_MODULE)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(diff_tracking main.cpp)

target_link_libraries(diff_tracking
        ${OpenCV_LIBS}
        Eigen3::Eigen)