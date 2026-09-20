#include "TransformTest.h"
#include "VisionTypes.h"
#include "CoordinateTransform.h"
#include "TransformUtils.h"

#include <iostream>


bool testTransform()
{
    // 1. 有效 Transform
    const TransformMatrix T_valid{ {
    {{0, -1, 0, 1.0}},
    {{1,  0, 0, 0.5}},
    {{0,  0, 1, 0.2}},
    {{0,  0, 0, 1.0}}
} };

    TransformMatrix T_inverse{};

    // 2. 求逆
    const bool success =
        TransformUtils::inverseTransform(
            T_valid,
            T_inverse);

    if (!success)
    {
        std::cout
            << "Transform inverse failed"
            << std::endl;

        return false;
    }

    // 3. Camera point
    const CameraPoint original{
        0.2,
        0.1,
        2.0
    };

    // 4. Camera → Robot
    const RobotPoint robot =
        CoordinateTransform::cameraToRobot(
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

    // 5. Robot → Camera
    const CameraPoint back =
        CoordinateTransform::robotToCamera(
            robot,
            T_inverse);

    std::cout
        << "Camera point back: "
        << back.X << ", "
        << back.Y << ", "
        << back.Z
        << std::endl;

    // 6. 检查 Round Trip
    if (!CoordinateTransform::isSamePoint(
        original,
        back,
        1e-6))
    {
        std::cout
            << "Round trip test: FAIL"
            << std::endl;

        return false;
    }

    // 7. 无效 Transform
    const TransformMatrix T_invalid{ {
    {{2, -1, 0, 1.0}},
    {{1,  0, 0, 0.5}},
    {{0,  0, 1, 0.2}},
    {{0,  0, 0, 1.0}}
} };

    TransformMatrix invalidInverse{};

    const bool invalidResult =
        TransformUtils::inverseTransform(
            T_invalid,
            invalidInverse);

    if (invalidResult)
    {
        std::cout
            << "Invalid transform test: FAIL"
            << std::endl;

        return false;
    }

    std::cout
        << "Invalid transform test: PASS"
        << std::endl;

    return true;
}


void printRotationComposition()
{
    RotationMatrix Rx{};
    RotationMatrix Ry{};
    RotationMatrix Rz{};

    TransformUtils::rotationX(30.0, Rx);
    TransformUtils::rotationY(20.0, Ry);
    TransformUtils::rotationZ(90.0, Rz);

    RotationMatrix temp{};
    RotationMatrix R_1{};

    // Rz × Ry × Rx
    TransformUtils::multiplyMatrix3x3(
        Ry,
        Rx,
        temp);

    TransformUtils::multiplyMatrix3x3(
        Rz,
        temp,
        R_1);

    std::cout
        << "Rz × Ry × Rx："
        << std::endl;

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            std::cout
                << R_1[row][col]
                << '\t';
        }

        std::cout << std::endl;
    }

    RotationMatrix temp2{};
    RotationMatrix R_2{};

    // Rx × Ry × Rz
    TransformUtils::multiplyMatrix3x3(
        Ry,
        Rz,
        temp2);

    TransformUtils::multiplyMatrix3x3(
        Rx,
        temp2,
        R_2);

    std::cout
        << "Rx × Ry × Rz："
        << std::endl;

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            std::cout
                << R_2[row][col]
                << '\t';
        }

        std::cout << std::endl;
    }
}


void printRotateRobotPoint(
    const CameraPoint& cameraPoint)
{
    RotationMatrix rotation{};
    TransformMatrix transform{};

    TransformUtils::rotationZ(
        90.0,
        rotation);

    TransformUtils::buildTransform(
        rotation,
        0.7,
        2.1,
        3.0,
        transform);

    const RobotPoint robotResult =
        CoordinateTransform::cameraToRobot(
            cameraPoint,
            transform);

    std::cout
        << "================"
        << std::endl;

    std::cout
        << "Rz + 平移后的机器人坐标："
        << std::endl;

    std::cout
        << "x: "
        << robotResult.X
        << std::endl;

    std::cout
        << "y: "
        << robotResult.Y
        << std::endl;

    std::cout
        << "z: "
        << robotResult.Z
        << std::endl;
}


bool demoLetterboxToCamera(
    const CameraConfig& config,
    CameraPoint& result)
{
    const double x = 450.0;
    const double y = 200.0;
    const double scale = 0.5;
    const double padX = 0.0;
    const double padY = 80.0;

    const Point2D restoredPoint =
        CoordinateTransform::restorePoint(
            { x, y },
            scale,
            padX,
            padY);

    std::cout
        << "================"
        << std::endl;

    std::cout
        << "还原后的 x："
        << restoredPoint.x
        << std::endl;

    std::cout
        << "还原后的 y："
        << restoredPoint.y
        << std::endl;

    const Target detectedTarget{
        100,
        0.95,
        restoredPoint.x,
        restoredPoint.y,
        false
    };

    CameraPoint cameraPointResult{};

    const bool cameraSuccess =
        CoordinateTransform::targetToCamera(
            detectedTarget,
            config.Z,
            config.fx,
            config.fy,
            config.cx,
            config.cy,
            cameraPointResult);

    if (!cameraSuccess)
    {
        return false;
    }

    std::cout
        << "================"
        << std::endl;

    std::cout
        << "Camera X："
        << cameraPointResult.X
        << std::endl;

    std::cout
        << "Camera Y："
        << cameraPointResult.Y
        << std::endl;

    std::cout
        << "Camera Z："
        << cameraPointResult.Z
        << std::endl;

    result = cameraPointResult;
    return true;
}