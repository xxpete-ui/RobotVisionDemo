#include "RobotVision.h"
#include <iostream>

bool testTransform()
{
    double T_valid[4][4] =
    {
        {0, -1, 0, 1.0},
        {1,  0, 0, 0.5},
        {0,  0, 1, 0.2},
        {0,  0, 0, 1.0}
    };

    double T_inverse[4][4];

    bool success = inverseTransform(T_valid, T_inverse);

    if (!success)
    {
        std::cout << "变换矩阵非法，无法求逆。" << std::endl;
        return false;
    }

    CameraPoint cameraPoint =
    {
        0.2,
        0.1,
        2.0
    };

    RobotPoint robotPoint =
        cameraToRobot(cameraPoint, T_valid);

    CameraPoint cameraPointBack =
        robotToCamera(robotPoint, T_inverse);

    std::cout << "Original camera point: "
        << cameraPoint.X << ", "
        << cameraPoint.Y << ", "
        << cameraPoint.Z << std::endl;

    std::cout << "Robot point: "
        << robotPoint.X << ", "
        << robotPoint.Y << ", "
        << robotPoint.Z << std::endl;

    std::cout << "Camera point back: "
        << cameraPointBack.X << ", "
        << cameraPointBack.Y << ", "
        << cameraPointBack.Z << std::endl;

    bool success_result =
        isSamePoint(cameraPoint, cameraPointBack, 1e-6);

    double T_invalid[4][4] =
    {
        {2, -1, 0, 1.0},
        {1,  0, 0, 0.5},
        {0,  0, 1, 0.2},
        {0,  0, 0, 1.0}
    };

    double T_invalid_inverse[4][4];

    bool invalidResult =
        inverseTransform(T_invalid, T_invalid_inverse);

    // 非法矩阵应该求逆失败
    if (invalidResult == false)
    {
        std::cout << "Invalid transform test: PASS" << std::endl;
    }
    else
    {
        std::cout << "Invalid transform test: FAIL" << std::endl;
        return false;
    }

    return success_result;
}