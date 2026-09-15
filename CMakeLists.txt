cmake_minimum_required(VERSION 3.15)

project(RobotVisionDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(/utf-8)

find_package(OpenCV REQUIRED)

add_executable(
    RobotVisionDemo
    src/main.cpp
    src/RobotVision.cpp
    src/TransformTest.cpp
)

target_include_directories(
    RobotVisionDemo
    PRIVATE
    include
)

target_link_libraries(
    RobotVisionDemo
    PRIVATE
    ${OpenCV_LIBS}
)