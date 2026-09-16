#include "RobotVision.h"
#include "TransformTest.h"
#include <iostream>


bool testTransform()
{
    // ================================
    // 1. 有效 Transform
    // ================================

    double T_valid[4][4] =
    {
        {0, -1, 0, 1.0},
        {1,  0, 0, 0.5},
        {0,  0, 1, 0.2},
        {0,  0, 0, 1.0}
    };


    double T_inverse[4][4];


    // ================================
    // 2. 求逆
    // ================================

    bool success =
        RobotVision::inverseTransform(
            T_valid,
            T_inverse);


    if (!success)
    {
        std::cout
            << "Transform inverse failed"
            << std::endl;

        return false;
    }


    // ================================
    // 3. Camera point
    // ================================

    CameraPoint original =
    {
        0.2,
        0.1,
        2.0
    };


    // ================================
    // 4. Camera → Robot
    // ================================

    RobotPoint robot =
        cameraToRobot(
            original,
            T_valid);


    std::cout
        << "Original camera point: "
        << original.X << ", "
        << original.Y << ", "
        << original.Z
        << std::endl;


    std::cout
        << "Robot point: "
        << robot.X << ", "
        << robot.Y << ", "
        << robot.Z
        << std::endl;


    // ================================
    // 5. Robot → Camera
    // ================================

    CameraPoint back =
        robotToCamera(
            robot,
            T_inverse);


    std::cout
        << "Camera point back: "
        << back.X << ", "
        << back.Y << ", "
        << back.Z
        << std::endl;


    // ================================
    // 6. 检查 Round Trip
    // ================================

    if (!isSamePoint(
        original,
        back,
        1e-6))
    {
        std::cout
            << "Round trip test: FAIL"
            << std::endl;

        return false;
    }


    // ================================
    // 7. 无效 Transform
    // ================================

    double T_invalid[4][4] =
    {
        {2, -1, 0, 1.0},
        {1,  0, 0, 0.5},
        {0,  0, 1, 0.2},
        {0,  0, 0, 1.0}
    };


    double invalidInverse[4][4];

    bool invalidResult = RobotVision::inverseTransform(
            T_invalid,
            invalidInverse);


    // 无效 Transform 应该求逆失败
    if (invalidResult)
    {
        std::cout << "Invalid transform test: FAIL" << std::endl;
        return false;
    }


    std::cout << "Invalid transform test: PASS" << std::endl;
    // ================================
    // 8. 所有测试通过
    // ================================

    return true;
}