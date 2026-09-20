cmake_minimum_required(VERSION 3.15)

project(RobotVisionDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (MSVC)
    add_compile_options(/utf-8)
endif()
find_package(OpenCV REQUIRED)
enable_testing()

add_library(RobotVisionCore STATIC
    src/RobotVision.cpp
    src/TargetProcessing.cpp
    src/CoordinateTransform.cpp
    src/Logger.cpp
    src/TransformUtils.cpp
 )

target_include_directories(RobotVisionCore PUBLIC include)

add_executable(
    RobotVisionDemo
    src/main.cpp
    src/TransformTest.cpp
    src/ImageDemo.cpp)

add_executable(RobotVisionStateTest
    tests/RobotVisionStateTest.cpp
 )

target_link_libraries(RobotVisionStateTest
    PRIVATE RobotVisionCore
)

function(add_vision_test test_name scenario label_name)
    add_test(
        NAME ${test_name}
        COMMAND RobotVisionStateTest ${scenario}
    )

    set_tests_properties(
        ${test_name}
        PROPERTIES LABELS ${label_name}
        TIMEOUT 5
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endfunction()


add_vision_test(testRecovery recovery state)
add_vision_test(testGrabbedTargetFiltering filtering state)

add_vision_test(
    testInvalidCameraConfig
    invalid-camera
    validation
)

add_vision_test(
    testInvalidTransform
    invalid-transform
    validation
)

add_vision_test(
    testSuccessfulPipeline
    pipeline
    transform
)

add_vision_test(
    testRotationTransform
    rotation
    transform
)

add_vision_test(
    testInverseTransformRoundTrip
    inverse
    transform
)

add_vision_test(
    testInvalidInverseTransform
    invalid-inverse
    validation
)

add_vision_test(
    testInvalidProjectionParameters
    invalid-projection
    validation
)

add_vision_test(
    testRotationMatrixGeneration
    rotation-matrix
    transform
)

add_vision_test(
    testRotationMatrixMultiplication
    matrix-multiply
    transform
)

add_vision_test(
    testTransformMatrixGeneration
    transform-matrix
    transform
)

add_vision_test(
    testInvalidHomogeneousRow
    invalid-homogeneous-row
    validation
)

add_vision_test(
    testValidProjectionParameters
    valid-projection
    transform
)

# Windows 下运行时自动复制 OpenCV Debug DLL
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

target_link_libraries(RobotVisionDemo
    PRIVATE
    RobotVisionCore
    ${OpenCV_LIBS}
)