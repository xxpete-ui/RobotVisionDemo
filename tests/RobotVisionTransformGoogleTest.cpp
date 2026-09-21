#include "RobotVision.h"
#include "TransformUtils.h"
#include "CoordinateTransform.h"
#include <gtest/gtest.h>
#include <cstddef>
#include <optional>
#include <vector>

namespace
{
    constexpr CameraConfig kDefaultCamera{
        2.0,
        800.0,
        800.0,
        640.0,
        360.0
    };

    constexpr TransformMatrix kTranslationTransform{ {
        {{1.0, 0.0, 0.0, 0.7}},
        {{0.0, 1.0, 0.0, 2.1}},
        {{0.0, 0.0, 1.0, 3.0}},
        {{0.0, 0.0, 0.0, 1.0}}
    } };

    constexpr TransformMatrix kRotationTransform{ {
        {{0.0, -1.0, 0.0, 0.7}},
        {{1.0,  0.0, 0.0, 2.1}},
        {{0.0,  0.0, 1.0, 3.0}},
        {{0.0,  0.0, 0.0, 1.0}}
    } };

    constexpr double kTolerance = 1e-9;

    void expectRotationMatrixNear(
        const RotationMatrix& actual,
        const RotationMatrix& expected,
        double tolerance = kTolerance)
    {
        for (std::size_t row = 0; row < actual.size(); ++row)
        {
            for (std::size_t col = 0; col < expected.size(); ++col)
            {
                SCOPED_TRACE(
                    ::testing::Message()
                    << "row = " << row << "col = " << col
                );

                EXPECT_NEAR(
                    actual[row][col],
                    expected[row][col],
                    tolerance
                );
            }
        }
    }

    void expectTransformMatrixNear(
        const TransformMatrix& actual,
        const TransformMatrix& expected,
        double tolerance = kTolerance)
    {
        for (std::size_t row = 0;
            row < actual.size();
            ++row)
        {
            for (std::size_t col = 0;
                col < actual[row].size();
                ++col)
            {
                SCOPED_TRACE(
                    ::testing::Message()
                    << "row = " << row
                    << ", col = " << col);

                EXPECT_NEAR(
                    actual[row][col],
                    expected[row][col],
                    tolerance);
            }
        }
    }
}

TEST(
    RobotVisionTransformTest,
    ConvertsPixelThroughFullPipeline)
{
    RobotVision vision(
        kDefaultCamera,
        kTranslationTransform);

    const std::vector<Target> targets{
        {
            1,
            0.90,
            720.0,
            400.0,
            false
        }
    };

    const std::optional<ValidTarget> result =
        vision.run(targets);

    ASSERT_TRUE(result.has_value());

    EXPECT_EQ(
        result->target.id,
        1);

    EXPECT_NEAR(
        result->cameraPoint.X,
        0.2,
        kTolerance);

    EXPECT_NEAR(
        result->cameraPoint.Y,
        0.1,
        kTolerance);

    EXPECT_NEAR(
        result->cameraPoint.Z,
        2.0,
        kTolerance);

    EXPECT_NEAR(
        result->robotPoint.X,
        0.9,
        kTolerance);

    EXPECT_NEAR(
        result->robotPoint.Y,
        2.2,
        kTolerance);

    EXPECT_NEAR(
        result->robotPoint.Z,
        5.0,
        kTolerance);

    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::OK);
}

TEST(
    TransformUtilsTest,
    GeneratesNinetyDegreeZRotation)
{
    const RotationMatrix rotation =
        TransformUtils::rotationZ(90.0);

    /*理论矩阵：
         0  -1   0
         1   0   0
         0   0   1
        由于cos(90°)可能得到约6.123e-17，
        所以使用EXPECT_NEAR。*/

    const RotationMatrix expected{ {
        {{0.0, -1.0, 0.0}},
        {{1.0,  0.0, 0.0}},
        {{0.0,  0.0, 1.0}}
    } };

    expectRotationMatrixNear(
        rotation,
        expected);
    EXPECT_TRUE(
        TransformUtils::isValidRotationMatrix(
            rotation));
}

TEST(
    TransformUtilsTest,
    MultipliesTwoNinetyDegreeRotations)
{
    const RotationMatrix rotation90 = TransformUtils::rotationZ(90.0);

    const RotationMatrix result = TransformUtils::multiplyMatrix3x3(rotation90, rotation90);

    /*Rz(90°) × Rz(90°) = Rz(180°)
        理论结果：
        -1   0   0
         0  -1   0
         0   0   1 */

    const RotationMatrix expected{ {
        {{-1.0,  0.0, 0.0}},
        {{ 0.0, -1.0, 0.0}},
        {{ 0.0,  0.0, 1.0}}
    } };

    expectRotationMatrixNear(result, expected);

    EXPECT_TRUE(
        TransformUtils::isValidRotationMatrix(result)
    );
}

TEST(
    TransformUtilsTest,
    BuildsTransformFromRotationAndTranslation)
{
    const RotationMatrix rotation =
        TransformUtils::rotationZ(90.0);

    const TransformMatrix transform =
        TransformUtils::buildTransform(
            rotation,
            0.7,
            2.1,
            3.0);

    const TransformMatrix expected{ {
        {{0.0, -1.0, 0.0, 0.7}},
        {{1.0,  0.0, 0.0, 2.1}},
        {{0.0,  0.0, 1.0, 3.0}},
        {{0.0,  0.0, 0.0, 1.0}}
    } };

    expectTransformMatrixNear(
        transform,
        expected);

    EXPECT_TRUE(
        TransformUtils::isValidTransformMatrix(
            transform));
}

TEST(
    RobotVisionTransformTest,
    AppliesRotationAndTranslationThroughPipeline)
{
    RobotVision vision(
        kDefaultCamera,
        kRotationTransform);

    const std::vector<Target> targets{
        {
            1,
            0.90,
            720.0,
            400.0,
            false
        }
    };

    const std::optional<ValidTarget> result =
        vision.run(targets);

    ASSERT_TRUE(result.has_value());

    EXPECT_EQ(
        result->target.id,
        1);

    // 像素坐标转相机坐标
    EXPECT_NEAR(
        result->cameraPoint.X,
        0.2,
        kTolerance);

    EXPECT_NEAR(
        result->cameraPoint.Y,
        0.1,
        kTolerance);

    EXPECT_NEAR(
        result->cameraPoint.Z,
        2.0,
        kTolerance);

    /*
        绕Z轴旋转90°：

        x' = -y = -0.1
        y' =  x =  0.2
        z' =  z =  2.0

        加上平移：

        X = -0.1 + 0.7 = 0.6
        Y =  0.2 + 2.1 = 2.3
        Z =  2.0 + 3.0 = 5.0
    */

    EXPECT_NEAR(
        result->robotPoint.X,
        0.6,
        kTolerance);

    EXPECT_NEAR(
        result->robotPoint.Y,
        2.3,
        kTolerance);

    EXPECT_NEAR(
        result->robotPoint.Z,
        5.0,
        kTolerance);

    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::OK);
}

TEST(
    TransformUtilsTest,
    InverseTransformRestoresOriginalPoint)
{
    const std::optional<TransformMatrix> inverse =
        TransformUtils::inverseTransform(
            kRotationTransform);

    ASSERT_TRUE(inverse.has_value());

    const CameraPoint originalCameraPoint{
        0.2,
        0.1,
        2.0
    };

    const RobotPoint robotPoint =
        CoordinateTransform::cameraToRobot(
            originalCameraPoint,
            kRotationTransform);

    const CameraPoint restoredCameraPoint =
        CoordinateTransform::robotToCamera(
            robotPoint,
            *inverse);

    EXPECT_NEAR(
        restoredCameraPoint.X,
        originalCameraPoint.X,
        kTolerance);

    EXPECT_NEAR(
        restoredCameraPoint.Y,
        originalCameraPoint.Y,
        kTolerance);

    EXPECT_NEAR(
        restoredCameraPoint.Z,
        originalCameraPoint.Z,
        kTolerance);
}

TEST(
    CoordinateTransformTest,
    ValidProjectionProducesExpectedCameraPoint)
{
    const Target target{
        1,
        0.90,
        720.0,
        400.0,
        false
    };

    const std::optional<CameraPoint> cameraPoint =
        CoordinateTransform::targetToCamera(
            target,
            kDefaultCamera);

    ASSERT_TRUE(
        cameraPoint.has_value());

    EXPECT_NEAR(
        cameraPoint->X,
        0.2,
        kTolerance);

    EXPECT_NEAR(
        cameraPoint->Y,
        0.1,
        kTolerance);

    EXPECT_NEAR(
        cameraPoint->Z,
        2.0,
        kTolerance);
}