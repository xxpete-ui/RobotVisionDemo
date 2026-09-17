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
    src/Logger.cpp
 )

target_include_directories(
    RobotVisionDemo
    PRIVATE
    include
)

# Windows 下运行时自动复制 OpenCV Debug DLL
if (WIN32)
    add_custom_command(TARGET RobotVisionDemo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "C:/OpenCV/opencv/build/x64/vc16/bin/opencv_world4140d.dll"
        "$<TARGET_FILE_DIR:RobotVisionDemo>"
    )
endif()

target_link_libraries(
    RobotVisionDemo
    PRIVATE
    ${OpenCV_LIBS}
)