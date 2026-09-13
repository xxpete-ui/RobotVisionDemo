cmake_minimum_required(VERSION 3.15)

project(RobotVisionDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(
    RobotVisionDemo
    src/main.cpp
    src/RobotVision.cpp
)

target_include_directories(
    RobotVisionDemo
    PRIVATE
    include
)