cmake_minimum_required(VERSION 3.15)

project(RobotVisionDemo)

option(
    ROBOTVISION_BUILD_DEMO
    "build the OpenCV demonstration executable"
    ON
)

option(
    ROBOTVISION_ENABLE_SANITIZERS
    "Enable AddressSanitizer and UndefinedBehaniorSanitizer"
    OFF
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (MSVC)
    add_compile_options(/utf-8)
endif()

enable_testing()

function(enable_project_warnings target_name)
    if (MSVC)
	target_compile_options(
	    ${target_name}
	    PRIVATE
	    /W4
	)
    else()
	target_compile_options(
	    ${target_name}
	    PRIVATE
	    -Wall
	    -Wextra
	    -Wpedantic
	)
    endif()
endfunction()

if (
    ROBOTVISION_ENABLE_SANITIZERS
    AND NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang"
)
    message(
        FATAL_ERROR
        "Sanitizers are currently supported only with GCC or Clang"
    )
endif()

function(enable_project_sanitizers target_name)
    if (NOT ROBOTVISION_ENABLE_SANITIZERS)
        return()
    endif()

    target_compile_options(
        ${target_name}
        PRIVATE
        -fsanitize=address,undefined
        -fno-omit-frame-pointer
    )

    get_target_property(
        target_type
        ${target_name}
        TYPE
    )

    if (NOT target_type STREQUAL "STATIC_LIBRARY")
        target_link_options(
            ${target_name}
            PRIVATE
            -fsanitize=address,undefined
        )
    endif()
endfunction()

include(FetchContent)

# Use the same dynamic MSVC runtime for GoogleTest and project targets
# This avoids /MT and /MD runtime mismatches
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
    src/MockDetector.cpp
    src/VisionPipeline.cpp
)

target_include_directories(
    RobotVisionCore
    PUBLIC
    include
)
enable_project_warnings(RobotVisionCore)
enable_project_sanitizers(RobotVisionCore)

if (ROBOTVISION_BUILD_DEMO)
    find_package(OpenCV REQUIRED)

    add_executable(
        RobotVisionDemo
        src/main.cpp
        src/TransformTest.cpp
        src/ImageDemo.cpp
        src/YoloDetector.cpp
    )
    enable_project_warnings(RobotVisionDemo)
    enable_project_sanitizers(RobotVisionDemo)

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

enable_project_warnings(
    RobotVisionStateGoogleTest
)

enable_project_sanitizers(
    RobotVisionStateGoogleTest
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

enable_project_warnings(
    RobotVisionValidationGoogleTest
)

enable_project_sanitizers(
    RobotVisionValidationGoogleTest
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

enable_project_warnings(
    RobotVisionTransformGoogleTest
)

enable_project_sanitizers(
    RobotVisionTransformGoogleTest
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

add_executable(
    MockDetectorGoogleTest
    tests/MockDetectorGoogleTest.cpp)

enable_project_warnings(
    MockDetectorGoogleTest
)

enable_project_sanitizers(
    MockDetectorGoogleTest
)

target_link_libraries(
    MockDetectorGoogleTest
    PRIVATE
    RobotVisionCore
    GTest::gtest_main
)

gtest_discover_tests(
    MockDetectorGoogleTest
    WORKING_DIRECTORY
        ${CMAKE_SOURCE_DIR}
    PROPERTIES
        LABELS detector
        TIMEOUT 5
)

add_executable(
    VisionPipelineGoogleTest
    tests/VisionPipelineGoogleTest.cpp
)

enable_project_warnings(
    VisionPipelineGoogleTest
)

enable_project_sanitizers(
    VisionPipelineGoogleTest
)

target_link_libraries(
    VisionPipelineGoogleTest
    PRIVATE
    RobotVisionCore
    GTest::gtest_main
)

gtest_discover_tests(
    VisionPipelineGoogleTest
    WORKING_DIRECTORY
        ${CMAKE_SOURCE_DIR}
    PROPERTIES
        LABELS detector
        TIMEOUT 5
)
