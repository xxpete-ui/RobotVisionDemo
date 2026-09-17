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

void printRotationComposition() {
    // ================================
   // 12. Rx + Ry + Rz
   // ================================

    double Rx[3][3];
    double Ry[3][3];
    double Rz[3][3];

    RobotVision::rotationX(
        30.0,
        Rx);

    RobotVision::rotationY(
        20.0,
        Ry);

    RobotVision::rotationZ(
        90.0,
        Rz);


    double temp[3][3];
    double R_1[3][3];


    // Rz × Ry × Rx

    RobotVision::multiplyMatrix3x3(
        Ry,
        Rx,
        temp);

    RobotVision::multiplyMatrix3x3(
        Rz,
        temp,
        R_1);


    std::cout
        << "Rz × Ry × Rx："
        << std::endl;


    for (int row = 0;
        row < 3;
        row++)
    {
        for (int col = 0;
            col < 3;
            col++)
        {
            std::cout
                << R_1[row][col]
                << "\t";
        }

        std::cout
            << std::endl;
    }


    // ================================
    // 13. Rx × Ry × Rz
    // ================================

    double temp2[3][3];
    double R_2[3][3];


    RobotVision::multiplyMatrix3x3(
        Ry,
        Rz,
        temp2);

    RobotVision::multiplyMatrix3x3(
        Rx,
        temp2,
        R_2);


    std::cout
        << "Rx × Ry × Rz："
        << std::endl;


    for (int row = 0;
        row < 3;
        row++)
    {
        for (int col = 0;
            col < 3;
            col++)
        {
            std::cout
                << R_2[row][col]
                << "\t";
        }

        std::cout
            << std::endl;
    }
    return;
}


void printRotateRobotPoint(const CameraPoint& cameraPoint) {
    // ================================
    // 11. Rz(90°) + 平移
    // ================================

    double T_1[4][4];
    double R[3][3];
    RobotVision::rotationZ(90.0, R);
    RobotVision::buildTransform(R, 0.7, 2.1, 3.0, T_1);
    RobotPoint robotResult = cameraToRobot(cameraPoint, T_1);
    std::cout << "================" << std::endl;
    std::cout << "Rz + 平移后的机器人坐标：" << std::endl;
    std::cout << "x: " << robotResult.X << std::endl;
    std::cout << "y: " << robotResult.Y << std::endl;
    std::cout << "z: " << robotResult.Z << std::endl;
    return;
}