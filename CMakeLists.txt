cmake_minimum_required(VERSION 3.15)

project(RobotVisionDemo)

option(
    ROBOTVISION_BUILD_DEMO
    "build the OpenCV demonstration executable"
    ON
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (MSVC)
    add_compile_options(/utf-8)
endif()

enable_testing()

include(FetchContent)

# Windows下让GoogleTest使用与主项目一致的动态CRT，
# 避免/MT、/MD运行库不一致导致链接问题
if (MSVC)
    set(
        gtest_force_shared_crt
        ON
        CACHE BOOL ""
        FORCE
    )
endif()

FetchContent_Declare(
    googletest
    GIT_REPOSITORY
        https://github.com/google/googletest.git
    GIT_TAG
        v1.18.0
    GIT_SHALLOW
        TRUE
)

FetchContent_MakeAvailable(googletest)

add_library(
    RobotVisionCore
    STATIC
    src/RobotVision.cpp
    src/TargetProcessing.cpp
    src/CoordinateTransform.cpp
    src/Logger.cpp
    src/TransformUtils.cpp
)

target_include_directories(
    RobotVisionCore
    PUBLIC
    include
)

if (ROBOTVISION_BUILD_DEMO)
    find_package(OpenCV REQUIRED)

    add_executable(
        RobotVisionDemo
        src/main.cpp
        src/TransformTest.cpp
        src/ImageDemo.cpp
    )

    target_link_libraries(
        RobotVisionDemo
        PRIVATE
        RobotVisionCore
        ${OpenCV_LIBS}
    )

    # Copy the OpenCV runtime DLL next to the demo executable on Windows.
    if (WIN32 AND TARGET opencv_world)
        add_custom_command(
            TARGET RobotVisionDemo
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:opencv_world>"
                "$<TARGET_FILE_DIR:RobotVisionDemo>"
            COMMENT "Copying OpenCV runtime DLL"
        )
    endif()
endif()


include(GoogleTest)

add_executable(
    RobotVisionStateGoogleTest
    tests/RobotVisionStateGoogleTest.cpp
)

target_link_libraries(
    RobotVisionStateGoogleTest
    PRIVATE
    RobotVisionCore
    GTest::gtest_main
)

gtest_discover_tests(
    RobotVisionStateGoogleTest
    WORKING_DIRECTORY
        ${CMAKE_SOURCE_DIR}
    PROPERTIES
        LABELS state
        TIMEOUT 5
)

add_executable(
    RobotVisionValidationGoogleTest
    tests/RobotVisionValidationGoogleTest.cpp
)

target_link_libraries(
    RobotVisionValidationGoogleTest
    PRIVATE
    RobotVisionCore
    GTest::gtest_main
)

gtest_discover_tests(
    RobotVisionValidationGoogleTest
    WORKING_DIRECTORY
        ${CMAKE_SOURCE_DIR}
    PROPERTIES
        LABELS validation
        TIMEOUT 5
)

add_executable(
    RobotVisionTransformGoogleTest
    tests/RobotVisionTransformGoogleTest.cpp
)

target_link_libraries(
    RobotVisionTransformGoogleTest
    PRIVATE
    RobotVisionCore
    GTest::gtest_main
)

gtest_discover_tests(
    RobotVisionTransformGoogleTest
    WORKING_DIRECTORY
        ${CMAKE_SOURCE_DIR}
    PROPERTIES
        LABELS transform
        TIMEOUT 5
)
