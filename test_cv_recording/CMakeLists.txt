cmake_minimum_required(VERSION 3.10)
project(cv_recording)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(cv_recording cv_recording.cpp)

target_link_libraries(cv_recording
        ${OpenCV_LIBS})

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message("Add extra link libraries for ${CMAKE_CXX_COMPILER_ID}")
    target_link_libraries(chase
            stdc++fs)
endif ()