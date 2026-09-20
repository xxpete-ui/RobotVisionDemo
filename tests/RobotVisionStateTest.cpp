#include "RobotVision.h"
#include "TargetProcessing.h"
#include "CoordinateTransform.h"
#include "TransformUtils.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>


// namespace： 匿名，这些名字只会在当前的.cpp文件中生效，不会污染项目其他源文件
namespace {
    // constexpr: 这些测试配置在编译期确定，运行时不应修改
    constexpr CameraConfig kDefaultCamera{
        2.0,
        800.0,
        800.0,
        640.0,
        360.0
    };

    constexpr TransformMatrix kIdentityTransform{ {
        {{1, 0, 0, 0}},
        {{0, 1, 0, 0}},
        {{0, 0, 1, 0}},
        {{0, 0, 0, 1}}
    } };

    constexpr TransformMatrix kInvalidTransform{ {
        {{0, 0, 0, 0}},
        {{0, 0, 0, 0}},
        {{0, 0, 0, 0}},
        {{0, 0, 0, 1}}
    } };

    bool nearlyEqual(double a, double b, double epsilon = 1e-9) {
        return std::abs(a - b) < epsilon;
    }

} // namespace


bool testRecovery() {
    RobotVision vision(kDefaultCamera, kIdentityTransform);
    std::vector<Target> targets{ {3, 0.80, 720, 400, true} };
    ValidTarget result{};

    if (vision.run(targets, result) ||
        vision.getStatus() != VisionStatus::NoValidTarget) {
        std::cerr << "FAIL: expected NoValidTarget\n";
        return false;
    }

    targets[0].grabbed = false;
    if (!vision.run(targets, result) ||
        vision.getStatus() != VisionStatus::OK ||
        result.target.id != 3) {
        std::cerr << "FAIL: expected recovery with target ID 3\n";
        return false;
    }

    return true;
}


bool testGrabbedTargetFiltering() {

    RobotVision vision(kDefaultCamera, kIdentityTransform);
    std::vector<Target> targets{
        {1, 0.72, 700, 380, false},
        {2, 0.75, 720, 400, false}
    };
    ValidTarget selected{};

    if (TargetProcessing::markTargetGrabbed(targets, 999) ||
        targets[0].grabbed || targets[1].grabbed) {
        std::cerr << "FAIL: unknown ID changed target state\n";
        return false;
    }

    if (!vision.run(targets, selected) || selected.target.id != 2) {
        std::cerr << "FAIL: expected target ID 2 first\n";
        return false;
    }

    if (!TargetProcessing::markTargetGrabbed(targets, selected.target.id) ||
        !targets[1].grabbed) {
        std::cerr << "FAIL: could not mark target ID 2\n";
        return false;
    }

    if (!vision.run(targets, selected) || selected.target.id != 1) {
        std::cerr << "FAIL: expected target ID 1 after grabbing ID 2\n";
        return false;
    }

    return true;
}


bool testInvalidCameraConfig() {
    //fx = 0,像素转相机坐标时就会发生除0，因此属于无效配置
    CameraConfig invalidCamera{
        2.0,
        0.0,
        800.0,
        640.0,
        360.0
    };

    RobotVision vision(invalidCamera, kIdentityTransform);

    std::vector<Target> targets{
        {1, 0.90, 720, 400, false}
    };

    ValidTarget result{};

    if (vision.getStatus() != VisionStatus::InvalidCameraConfig) {
        std::cerr << "FAIL: expected InvalidCameraConfig after construction\n";
        return false;
    }

    if (vision.run(targets, result)) {
        std::cerr
            << "FAIL: run succeeded with invalid camera config\n";
        return false;
    }

    if (vision.getStatus() != VisionStatus::InvalidCameraConfig) {
        std::cerr
            << "FAIL: invalid camera status was not preserved\n";
        return false;
    }

    return true;
}


bool testInvalidTransform() {

    RobotVision vision(kDefaultCamera, kInvalidTransform);

    std::vector<Target> targets{
        {1, 0.90, 720, 400, false}
    };

    ValidTarget result{};

    if (vision.getStatus() != VisionStatus::InvalidTransform) {
        std::cerr
            << "FAIL: expected InvalidTransform after construction\n";
        return false;
    }

    if (vision.run(targets, result)) {
        std::cerr
            << "FAIL: run succeeded with invalid transform\n";
        return false;
    }

    // run() 不能清除这个永久配置错误
    if (vision.getStatus() != VisionStatus::InvalidTransform) {
        std::cerr
            << "FAIL: invalid transform status was not preserved\n";
        return false;
    }
    return true;
}


bool testSuccessfulPipeline() {
    const TransformMatrix transform = { {
        {{1, 0, 0, 0.7}},
        {{0, 1, 0, 2.1}},
        {{0, 0, 1, 3.0}},
        {{0, 0, 0, 1.0}}
    } };

    RobotVision vision(kDefaultCamera, transform);

    std::vector<Target> targets{
        {1, 0.90, 720, 400, false}
    };
    ValidTarget result{};

    if (!vision.run(targets, result)) {
        std::cerr << "FAIL: valid pipeline did not run\n";
        return false;
    }

    if (vision.getStatus() != VisionStatus::OK) {
        std::cerr << "FAIL: expected OK after successful run\n";
        return false;
    }

    if (result.target.id != 1) {
        std::cerr << "FAIL: unexpected selected target\n";
        return false;
    }

    if (!nearlyEqual(result.cameraPoint.X, 0.2) ||
        !nearlyEqual(result.cameraPoint.Y, 0.1) ||
        !nearlyEqual(result.cameraPoint.Z, 2.0)) {
        std::cerr << "FAIL: incorrect camera coordinates\n";
        return false;
    }

    if (!nearlyEqual(result.robotPoint.X, 0.9) ||
        !nearlyEqual(result.robotPoint.Y, 2.2) ||
        !nearlyEqual(result.robotPoint.Z, 5.0)) {
        std::cerr << "FAIL: incorrect translated robot coordinates\n";
        return false;
    }

    return true;
}


bool testRotationTransform() {

    // 绕 Z 轴旋转 90°，然后平移 (0.7, 2.1, 3.0)
    const TransformMatrix transform = { {
        {{0, -1, 0, 0.7}},
        {{1,  0, 0, 2.1}},
        {{0,  0, 1, 3.0}},
        {{0,  0, 0, 1.0}}
    } };

    RobotVision vision(kDefaultCamera, transform);

    std::vector<Target> targets{
        {1, 0.90, 720, 400, false}
    };
    ValidTarget result{};

    if (!vision.run(targets, result)) {
        std::cerr << "FAIL: rotation pipeline did not run\n";
        return false;
    }

    // 像素坐标转相机坐标仍为 (0.2, 0.1, 2.0)
    if (!nearlyEqual(result.cameraPoint.X, 0.2) ||
        !nearlyEqual(result.cameraPoint.Y, 0.1) ||
        !nearlyEqual(result.cameraPoint.Z, 2.0)) {
        std::cerr << "FAIL: incorrect camera point before rotation\n";
        return false;
    }

    /*
        Rz(90°)：

        x' = -y = -0.1
        y' =  x =  0.2
        z' =  z =  2.0

        加上平移：

        X = -0.1 + 0.7 = 0.6
        Y =  0.2 + 2.1 = 2.3
        Z =  2.0 + 3.0 = 5.0
    */
    if (!nearlyEqual(result.robotPoint.X, 0.6) ||
        !nearlyEqual(result.robotPoint.Y, 2.3) ||
        !nearlyEqual(result.robotPoint.Z, 5.0)) {
        std::cerr << "FAIL: incorrect rotated robot coordinates\n";
        return false;
    }

    return true;
}


bool testInverseTransformRoundTrip() {
    const TransformMatrix transform = { {
        {{0, -1, 0, 0.7}},
        {{1,  0, 0, 2.1}},
        {{0,  0, 1, 3.0}},
        {{0,  0, 0, 1.0}}
    } };

    TransformMatrix inverse{};

    if (!TransformUtils::inverseTransform(transform, inverse)) {
        std::cerr << "FAIL: could not invert valid transform\n";
        return false;
    }

    const CameraPoint originalCamera{
        0.2,
        0.1,
        2.0
    };

    const RobotPoint robotPoint = CoordinateTransform::cameraToRobot(originalCamera, transform);

    const CameraPoint restoredCamera = CoordinateTransform::robotToCamera(robotPoint, inverse);

    if (!nearlyEqual(restoredCamera.X, originalCamera.X) ||
        !nearlyEqual(restoredCamera.Y, originalCamera.Y) ||
        !nearlyEqual(restoredCamera.Z, originalCamera.Z)) {
        std::cerr << "FAIL: inverse transform did not restore camera point\n";
        return false;
    }

    return true;
}

bool testInvalidInverseTransform() {
    TransformMatrix inverse{};

    if (TransformUtils::inverseTransform(kInvalidTransform, inverse)) {
        std::cerr << "FAIL: invalid transform was inversed successful\n";
        return false;
    }
    return true;
}


bool testInvalidProjectionParameters()
{
    const Target target{
        1,
        0.90,
        720,
        400,
        false
    };

    CameraPoint result{};

    const bool converted = CoordinateTransform::targetToCamera(
        target,
        2.0,
        0.0,
        800.0,
        640.0,
        360.0,
        result
    );

    if (converted) {
        std::cerr << "FAIL: targetToCamera accepted fx = 0\n";
        return false;
    }

    return true;

}


bool testRotationMatrixGeneration() {
    RotationMatrix rotation{};

    TransformUtils::rotationZ(
        90.0,
        rotation
    );
    /*
        理论上的 Rz(90°)：

        0  -1   0
        1   0   0
        0   0   1

        cos(90°) 实际可能是 6.123e-17，
        因此使用 nearlyEqual() 比较。
    */

    if (!nearlyEqual(rotation[0][0], 0.0) ||
        !nearlyEqual(rotation[0][1], -1.0) ||
        !nearlyEqual(rotation[0][2], 0.0) ||
        !nearlyEqual(rotation[1][0], 1.0) ||
        !nearlyEqual(rotation[1][1], 0.0) ||
        !nearlyEqual(rotation[1][2], 0.0) ||
        !nearlyEqual(rotation[2][0], 0.0) ||
        !nearlyEqual(rotation[2][1], 0.0) ||
        !nearlyEqual(rotation[2][2], 1.0)) {
        std::cerr
            << "FAIL: incorrect Z rotation matrix\n";
        return false;
    }

    if (!TransformUtils::isValidRotationMatrix(rotation)) {
        std::cerr << "FAIL: generated rotation matrix is invalid\n";
        return false;
    }

    return true;
}


bool testRotationMatrixMultiplication() {
    const RotationMatrix identity{ {
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}}
    } };

    RotationMatrix rotation{};
    TransformUtils::rotationZ(90.0, rotation);

    RotationMatrix result{};

    TransformUtils::multiplyMatrix3x3(identity, rotation, result);
    //I × R应该等于R
    for (std::size_t row = 0; row < result.size(); row++) {
        for (std::size_t col = 0; col < result[row].size(); ++col) {
            if (!nearlyEqual(result[row][col], rotation[row][col])) {
                std::cerr << "FAIL: incorrect matrix multiplication\n";
                return false;
            }
        }
    }
    return true;
}


bool testTransformMatrixGeneration() {
    RotationMatrix rotation{};
    TransformUtils::rotationZ(90.0, rotation);

    TransformMatrix transform{};

    TransformUtils::buildTransform(rotation, 0.7, 2.1, 3.0, transform);

    const TransformMatrix excepeted{ {
        {{0.0, -1.0, 0.0, 0.7}},
        {{1.0, 0.0, 0.0, 2.1}},
        {{0.0, 0.0, 1.0, 3.0}},
        {{0.0, 0.0, 0.0, 1.0}}
} };

    for (std::size_t row = 0; row < transform.size(); ++row)
    {
        for (std::size_t col = 0; col < transform[row].size(); ++col)
        {
            if (!nearlyEqual(transform[row][col], excepeted[row][col]))
            {
                std::cerr << "FAIL: incorrect transform element at [" << row << "][" << col << "]\n";
                return false;
            }
        }
    }
    return true;
}


bool testInvalidHomogeneousRow() {
    const TransformMatrix transform{ {
        {{1.0, 0.0, 0.0, 0.7}},
        {{0.0, 1.0, 0.0, 2.1}},
        {{0.0, 0.0, 1.0, 3.0}},
        {{0.0, 0.0, 0.0, 2.0}}
} };

    if (TransformUtils::isValidTransformMatrix(transform)) {
        std::cerr << "FAIL: invalid homogeneous row was accepted\n";
        return false;
    }

    TransformMatrix inverse{};

    if (TransformUtils::inverseTransform(transform, inverse)) {
        std::cerr << "FAIL: inverted transform with invalid homogeneous row\n";
        return false;
    }

    return true;
}


int main(int argc, char* argv[])
{
    struct TestScenario
    {
        const char* name;
        bool (*function)(); // 指针函数，只能指向返回类型为bool，不接收参数的函数
    };

    const TestScenario scenarios[] = {
        {"recovery", testRecovery},
        {"filtering", testGrabbedTargetFiltering},
        {"invalid-camera", testInvalidCameraConfig},
        {"invalid-transform", testInvalidTransform},
        {"pipeline", testSuccessfulPipeline},
        {"rotation", testRotationTransform},
        {"inverse", testInverseTransformRoundTrip},
        {"invalid-inverse", testInvalidInverseTransform},
        {"invalid-projection", testInvalidProjectionParameters},
        {"rotation-matrix", testRotationMatrixGeneration},
        {"matrix-multiply", testRotationMatrixMultiplication},
        {"transform-matrix", testTransformMatrixGeneration},
        {"invalid-homogeneous-row", testInvalidHomogeneousRow}
    };

    if (argc != 2) {
        std::cerr << "Usage: RobotVisionStateTest <scenario>\n";
        std::cerr << "Available scenarios:\n";

        for (const auto& item : scenarios) {
            std::cerr << "  " << item.name << '\n';
        }

        return 2;
    }

    const std::string requestedScenario = argv[1];

    for (const auto& item : scenarios) {
        if (requestedScenario == item.name) {
            return item.function() ? 0 : 1;
        }
    }

    std::cerr
        << "Unknown scenario: "
        << requestedScenario
        << '\n';

    return 2;
}