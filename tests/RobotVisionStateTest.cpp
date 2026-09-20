#include "RobotVision.h"
#include "TargetProcessing.h"
#include "CoordinateTransform.h"
#include "TransformUtils.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>


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
    const auto missingResult = vision.run(targets);

    if (missingResult ||
        vision.getStatus() != VisionStatus::NoValidTarget)
    {
        std::cerr << "FAIL: expected NoValidTarget\n";
        return false;
    }

    targets[0].grabbed = false;

    const auto recoveredResult =
        vision.run(targets);

    if (!recoveredResult ||
        vision.getStatus() != VisionStatus::OK ||
        recoveredResult->target.id != 3)
    {
        std::cerr
            << "FAIL: expected recovery with target ID 3\n";
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
    const auto firstSelected =
        vision.run(targets);

    if (!firstSelected ||
        firstSelected->target.id != 2)
    {
        std::cerr
            << "FAIL: expected target ID 2 first\n";
        return false;
    }

    if (!TargetProcessing::markTargetGrabbed(
        targets,
        firstSelected->target.id) ||
        !targets[1].grabbed)
    {
        std::cerr
            << "FAIL: could not mark target ID 2\n";
        return false;
    }

    const auto secondSelected =
        vision.run(targets);

    if (!secondSelected ||
        secondSelected->target.id != 1)
    {
        std::cerr
            << "FAIL: expected target ID 1 after grabbing ID 2\n";
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

    if (vision.getStatus() != VisionStatus::InvalidCameraConfig) {
        std::cerr << "FAIL: expected InvalidCameraConfig after construction\n";
        return false;
    }

    if (vision.run(targets)) {
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

    if (vision.getStatus() != VisionStatus::InvalidTransform) {
        std::cerr
            << "FAIL: expected InvalidTransform after construction\n";
        return false;
    }

    if (vision.run(targets)) {
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
    const auto result = vision.run(targets);

    if (!result)
    {
        std::cerr
            << "FAIL: valid pipeline did not run\n";
        return false;
    }

    if (vision.getStatus() != VisionStatus::OK) {
        std::cerr << "FAIL: expected OK after successful run\n";
        return false;
    }

    if (result->target.id != 1) {
        std::cerr << "FAIL: unexpected selected target\n";
        return false;
    }

    if (!nearlyEqual(result->cameraPoint.X, 0.2) ||
        !nearlyEqual(result->cameraPoint.Y, 0.1) ||
        !nearlyEqual(result->cameraPoint.Z, 2.0)) {
        std::cerr << "FAIL: incorrect camera coordinates\n";
        return false;
    }

    if (!nearlyEqual(result->robotPoint.X, 0.9) ||
        !nearlyEqual(result->robotPoint.Y, 2.2) ||
        !nearlyEqual(result->robotPoint.Z, 5.0)) {
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
    const auto result =
        vision.run(targets);

    if (!result)
    {
        std::cerr
            << "FAIL: rotation pipeline did not run\n";
        return false;
    }

    // 像素坐标转相机坐标仍为 (0.2, 0.1, 2.0)
    if (!nearlyEqual(result->cameraPoint.X, 0.2) ||
        !nearlyEqual(result->cameraPoint.Y, 0.1) ||
        !nearlyEqual(result->cameraPoint.Z, 2.0)) {
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
    if (!nearlyEqual(result->robotPoint.X, 0.6) ||
        !nearlyEqual(result->robotPoint.Y, 2.3) ||
        !nearlyEqual(result->robotPoint.Z, 5.0)) {
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

    const std::optional<TransformMatrix> inverse =
        TransformUtils::inverseTransform(transform);

    if (!inverse)
    {
        std::cerr << "FAIL: could not invert valid transform\n";
        return false;
    }

    const CameraPoint originalCamera{
        0.2,
        0.1,
        2.0
    };

    const RobotPoint robotPoint = CoordinateTransform::cameraToRobot(originalCamera, transform);

    const CameraPoint restoredCamera = CoordinateTransform::robotToCamera(robotPoint, *inverse);

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

    if (TransformUtils::inverseTransform(kInvalidTransform))
    {
        std::cerr << "FAIL: invalid transform was inversed successfully\n";
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

    const CameraConfig config{
    2.0,
    0.0,
    800.0,
    640.0,
    360.0
    };

    const auto converted =
        CoordinateTransform::targetToCamera(
            target,
            config);

    if (converted) {
        std::cerr << "FAIL: targetToCamera accepted fx = 0\n";
        return false;
    }

    return true;

}

bool testRotationMatrixGeneration() {
    const RotationMatrix rotation = TransformUtils::rotationZ(90.0);
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

    const RotationMatrix rotation = TransformUtils::rotationZ(90.0);

    const RotationMatrix result = TransformUtils::multiplyMatrix3x3(identity, rotation);
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
    const RotationMatrix rotation = TransformUtils::rotationZ(90.0);

    TransformMatrix transform = TransformUtils::buildTransform(rotation, 0.7, 2.1, 3.0);

    const TransformMatrix expected{ {
        {{0.0, -1.0, 0.0, 0.7}},
        {{1.0, 0.0, 0.0, 2.1}},
        {{0.0, 0.0, 1.0, 3.0}},
        {{0.0, 0.0, 0.0, 1.0}}
} };

    for (std::size_t row = 0; row < transform.size(); ++row)
    {
        for (std::size_t col = 0; col < transform[row].size(); ++col)
        {
            if (!nearlyEqual(transform[row][col], expected[row][col]))
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

    if (TransformUtils::inverseTransform(transform))
    {
        std::cerr
            << "FAIL: inverted transform with invalid homogeneous row\n";
        return false;
    }

    return true;
}


bool testValidProjectionParameters() {
    const Target target
    {
        1,
        0.90,
        720.0,
        400.0,
        false
    };

    const CameraConfig config{
    2.0,
    800.0,
    800.0,
    640.0,
    360.0
    };

    const auto cameraPoint =
        CoordinateTransform::targetToCamera(
            target,
            config);

    if (!cameraPoint) {
        std::cerr << "FAIL: valid peojection returnen no value\n";
        return false;
    }

    if (!nearlyEqual(cameraPoint->X, 0.2) ||
        !nearlyEqual(cameraPoint->Y, 0.1) ||
        !nearlyEqual(cameraPoint->Z, 2.0))
    {
        std::cerr << "FAIL: valid projection returned incorrect coordonates\n";
        return false;
    }

    return true;
}


bool testNonFiniteProjectionParameters()
{
    const Target validTarget{
        1,
        0.90,
        720.0,
        400.0,
        false
    };
    // 获取double类型的 +∞ 和 NaN
    const double infinity =
        std::numeric_limits<double>::infinity();

    const double notANumber =
        std::numeric_limits<double>::quiet_NaN();

    const CameraConfig infiniteConfig{
        2.0,
        infinity,
        800.0,
        640.0,
        360.0
    };

    if (CoordinateTransform::targetToCamera(
        validTarget,
        infiniteConfig))
    {
        std::cerr
            << "FAIL: infinite focal length was accepted\n";
        return false;
    }

    const Target invalidTarget{
        2,
        0.90,
        notANumber, // target.x = NaN 非法像素坐标
        400.0,
        false
    };

    if (CoordinateTransform::targetToCamera(
        invalidTarget,
        kDefaultCamera))
    {
        std::cerr
            << "FAIL: NaN target coordinate was accepted\n";
        return false;
    }

    return true;
}


bool testNonFiniteTransform()
{
    const double infinity =
        std::numeric_limits<double>::infinity();

    const TransformMatrix transform{ {
        {{1.0, 0.0, 0.0, infinity}},
        {{0.0, 1.0, 0.0, 0.0}},
        {{0.0, 0.0, 1.0, 0.0}},
        {{0.0, 0.0, 0.0, 1.0}}
    } };

    if (TransformUtils::isValidTransformMatrix(transform))
    {
        std::cerr
            << "FAIL: transform with infinite translation was accepted\n";
        return false;
    }

    if (TransformUtils::inverseTransform(transform))
    {
        std::cerr
            << "FAIL: transform with infinite translation was inverted\n";
        return false;
    }

    return true;
}


bool testNonFiniteCameraConfig()
{
    const CameraConfig invalidCamera{
        2.0,
        std::numeric_limits<double>::infinity(),
        800.0,
        640.0,
        360.0
    };

    RobotVision vision(
        invalidCamera,
        kIdentityTransform);

    if (vision.getStatus() !=
        VisionStatus::InvalidCameraConfig)
    {
        std::cerr
            << "FAIL: infinite camera config was accepted\n";
        return false;
    }

    const std::vector<Target> targets{
        {1, 0.90, 720.0, 400.0, false}
    };

    if (vision.run(targets))
    {
        std::cerr
            << "FAIL: run succeeded with infinite camera config\n";
        return false;
    }

    if (vision.getStatus() !=
        VisionStatus::InvalidCameraConfig)
    {
        std::cerr
            << "FAIL: invalid camera status was not preserved\n";
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
        {"invalid-homogeneous-row", testInvalidHomogeneousRow},
        {"valid-projection", testValidProjectionParameters},
        {"non-finite-projection", testNonFiniteProjectionParameters},
        {"non-finite-transform", testNonFiniteTransform},
        {"non-finite-camera", testNonFiniteCameraConfig}
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