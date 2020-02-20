cmake_minimum_required(VERSION 3.10)
project(indoor_coordinates)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package (Eigen3 REQUIRED NO_MODULE)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(indoor_coordinates main.cpp)

target_link_libraries(indoor_coordinates
        ${OpenCV_LIBS}
        Eigen3::Eigen)