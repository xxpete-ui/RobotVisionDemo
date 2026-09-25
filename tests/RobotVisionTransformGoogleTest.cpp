#include "RobotVision.h"
#include "TransformUtils.h"
#include "CoordinateTransform.h"
#include "LetterboxGeometry.h"
#include "DepthSampler.h"
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

TEST(
    CoordinateTransformTest,
    UsesEachTargetsOwnDepth)
{
    // 两个目标像素位置相同，只改变各自的深度。
    Target nearTarget{ 1, 0.90, 720.0, 400.0, false };
    nearTarget.depthMeters = 1.0;

    Target farTarget{ 2, 0.90, 720.0, 400.0, false };
    farTarget.depthMeters = 3.0;

    const auto nearPoint =
        CoordinateTransform::targetToCamera(
            nearTarget, kDefaultCamera);

    const auto farPoint =
        CoordinateTransform::targetToCamera(
            farTarget, kDefaultCamera);

    ASSERT_TRUE(nearPoint.has_value());
    ASSERT_TRUE(farPoint.has_value());

    // 相机内参：fx=fy=800、cx=640、cy=360。
    // 像素偏移为 (80,40)；深度越大，反投影的 X/Y 也越大。
    EXPECT_NEAR(nearPoint->X, 0.1, kTolerance);
    EXPECT_NEAR(nearPoint->Y, 0.05, kTolerance);
    EXPECT_NEAR(nearPoint->Z, 1.0, kTolerance);

    EXPECT_NEAR(farPoint->X, 0.3, kTolerance);
    EXPECT_NEAR(farPoint->Y, 0.15, kTolerance);
    EXPECT_NEAR(farPoint->Z, 3.0, kTolerance);
}

TEST(
    CoordinateTransformValidationTest,
    RejectsInvalidTargetDepth)
{
    Target target{ 1, 0.90, 720.0, 400.0, false };

    // 有值但为 0：应拒绝，不能悄悄回退到 config.Z。
    target.depthMeters = 0.0;
    EXPECT_FALSE(
        CoordinateTransform::targetToCamera(
            target, kDefaultCamera).has_value());

    // NaN 和无穷大同样无效。
    target.depthMeters =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(
        CoordinateTransform::targetToCamera(
            target, kDefaultCamera).has_value());

    target.depthMeters =
        std::numeric_limits<double>::infinity();
    EXPECT_FALSE(
        CoordinateTransform::targetToCamera(
            target, kDefaultCamera).has_value());
}

TEST(LetterboxGeometryTest, RestoresBoxToOriginalImage)
{
    // 1920×1080 → 640×360，上下各填 140。
    // 模型框 (243,283) 到 (269,307)
    // 应还原为原图框 (729,429,78,72)。
    // 带F表示浮点数32位，double是64位
    const auto box = LetterboxGeometry::restoreBox(
        243.0F, 283.0F, 269.0F, 307.0F,
        1.0F / 3.0F,
        0, 140,
        1920, 1080);

    ASSERT_TRUE(box.has_value());
    EXPECT_NEAR(box->x, 729.0, 0.001);
    EXPECT_NEAR(box->y, 429.0, 0.001);
    EXPECT_NEAR(box->width, 78.0, 0.001);
    EXPECT_NEAR(box->height, 72.0, 0.001);
}

TEST(LetterboxGeometryTest, ClipsBoxAtImageEdge)
{
    // 左边预测到了图像外，裁剪后从原图 x=0 开始。
    const auto box = LetterboxGeometry::restoreBox(
        -5.0F, 140.0F, 10.0F, 160.0F,
        1.0F / 3.0F,
        0, 140,
        1920, 1080);

    ASSERT_TRUE(box.has_value());
    EXPECT_DOUBLE_EQ(box->x, 0.0);
    EXPECT_NEAR(box->width, 30.0, 0.001);
}

TEST(LetterboxGeometryTest, RejectsBoxOutsideImage)
{
    // 左右边都在原图左侧，clamp 后宽度为 0。
    const auto box = LetterboxGeometry::restoreBox(
        -10.0F, 140.0F, -1.0F, 160.0F,
        1.0F / 3.0F,
        0, 140,
        1920, 1080);

    EXPECT_FALSE(box.has_value());
}

TEST(
    RobotVisionTransformTest,
    SelectsValidTargetWithItsOwnDepth)
{
    RobotVision vision(kDefaultCamera, kTranslationTransform);

    Target invalidTarget{ 1, 0.99, 720.0, 400.0, false };
    invalidTarget.depthMeters = 0.0; // 置信度最高，但深度无效。

    Target validTarget{ 2, 0.80, 720.0, 400.0, false };
    validTarget.depthMeters = 3.0;

    const std::vector<Target> targets{
        invalidTarget,
        validTarget
    };

    const std::optional<ValidTarget> result =
        vision.run(targets);

    ASSERT_TRUE(result.has_value());

    // 目标 1 被跳过，选择目标 2。
    EXPECT_EQ(result->target.id, 2);
    EXPECT_EQ(vision.getStatus(), VisionStatus::OK);

    // 像素偏移 (80,40)、深度 3m → 相机点 (0.3,0.15,3)。
    EXPECT_NEAR(result->cameraPoint.X, 0.3, kTolerance);
    EXPECT_NEAR(result->cameraPoint.Y, 0.15, kTolerance);
    EXPECT_NEAR(result->cameraPoint.Z, 3.0, kTolerance);

    // 测试矩阵再平移 (0.7,2.1,3)。
    EXPECT_NEAR(result->robotPoint.X, 1.0, kTolerance);
    EXPECT_NEAR(result->robotPoint.Y, 2.25, kTolerance);
    EXPECT_NEAR(result->robotPoint.Z, 6.0, kTolerance);
}

TEST(DepthSamplerTest, ReadsMillimetersAsMeters)
{
    // 2 行3 列：
    // 第 0 行：1000, 2000, 3000 mm
    // 第 1 行：4000,    0, 6000 mm
    // A赋值给width，B赋值给height，C用来构造depthMillimeters这个vector
    const DepthFrame frame{
        3,
        2,
        {1000, 2000, 3000, 4000, 0, 6000}
    };

    const auto depth = DepthSampler::sampleMeters(
        frame, 1.0, 0.0);

    ASSERT_TRUE(depth.has_value());
    EXPECT_DOUBLE_EQ(*depth, 2.0);
}

TEST(DepthSamplerTest, RejectsMissingDepthAndOutOfBoundsCenter)
{
    const DepthFrame frame{
        3,
        2,
        {1000, 2000, 3000,
         4000,    0, 6000}
    };

    // 像素索引公式：index = y * width + x
    // 第 1 行、第 1 列的值为 0：表示没有有效深度。
    EXPECT_FALSE(
        DepthSampler::sampleMeters(
            frame, 1.0, 1.0).has_value());

    // 宽度为 3，合法列索引只有 0、1、2。
    EXPECT_FALSE(
        DepthSampler::sampleMeters(
            frame, 3.0, 0.0).has_value());

    // 负坐标也不能用于访问数组。
    EXPECT_FALSE(
        DepthSampler::sampleMeters(
            frame, -1.0, 0.0).has_value());
}

TEST(DepthSamplerTest, RoundsCenterToNearestPixel)
{
    const DepthFrame frame{
        3,
        2,
        {1000, 2000, 3000,
         4000,    0, 6000}
    };

    // 像素索引公式：index = y * width + x
    // (1.6, 1.0) 四舍五入为第 1 行、第 2 列，即 6000 mm。
    const auto depth =
        DepthSampler::sampleMeters(
            frame, 1.6, 1.0);

    ASSERT_TRUE(depth.has_value());
    EXPECT_DOUBLE_EQ(*depth, 6.0);
}

TEST(
    RobotVisionTransformTest,
    UsesSampledDepthFromFrame)
{
    // 第 0 行第 1 列是 2000 mm，也就是 2 m。
    const DepthFrame depthFrame{
        3, // X
        2, // y
        {1000, 2000, 3000,
         4000,    0, 6000}
    };

    Target target{ 1, 0.90, 1.0, 0.0, false };

    // 1. 按目标中心在深度图中取值。
    const std::optional<double> sampledDepth =
        DepthSampler::sampleMeters(
            depthFrame,
            target.x,
            target.y);

    ASSERT_TRUE(sampledDepth.has_value());

    // 2. 把采到的米数交给这个目标。
    target.depthMeters = sampledDepth;

    // 小尺寸测试图使用对应的测试内参：
    // fx=fy=2，主点=(0,0)。
    const CameraConfig camera{
        2.0, 2.0, 2.0, 0.0, 0.0
    };

    RobotVision vision(camera, kTranslationTransform);

    // 3. RobotVision 读取 target.depthMeters 完成投影。
    const std::optional<ValidTarget> result =
        vision.run({ target });

    ASSERT_TRUE(result.has_value());

    // 像素 (1,0)、深度 2m：
    // 相机点 X=(1-0)*2/2=1，Y=0，Z=2。
    EXPECT_NEAR(result->cameraPoint.X, 1.0, kTolerance);
    EXPECT_NEAR(result->cameraPoint.Y, 0.0, kTolerance);
    EXPECT_NEAR(result->cameraPoint.Z, 2.0, kTolerance);

    // 再经过测试矩阵平移 (0.7,2.1,3)。
    EXPECT_NEAR(result->robotPoint.X, 1.7, kTolerance);
    EXPECT_NEAR(result->robotPoint.Y, 2.1, kTolerance);
    EXPECT_NEAR(result->robotPoint.Z, 5.0, kTolerance);
}

TEST(DepthSamplerTest, KeepsOnlyTargetsWithValidDepth)
{
    const DepthFrame frame{
        3,
        2,
        {1000, 2000, 3000,
         4000,    0, 6000}
    };

    const std::vector<Target> detected{
        {1, 0.90, 0.0, 0.0, false}, // 1000 mm，有效
        {2, 0.80, 3.0, 1.0, false}, // 0 mm，无效
        {3, 0.70, 3.0, 0.0, false}  // 越界
    };

    const std::vector<Target> result =
        DepthSampler::attachValidDepth(detected, frame);

    ASSERT_EQ(result.size(), 1U);
    EXPECT_EQ(result[0].id, 1);
    EXPECT_DOUBLE_EQ(result[0].confidence, 0.90);

    ASSERT_TRUE(result[0].depthMeters.has_value());
    EXPECT_DOUBLE_EQ(*result[0].depthMeters, 1.0);

    // 原始检测结果没有被修改。
    EXPECT_FALSE(detected[0].depthMeters.has_value());
}