#include "RobotVision.h"
#include "FrameSync.h"
#include <gtest/gtest.h>
#include <optional>
#include <vector>
#include <limits>
#include <string>
#include <ostream>
#include "TransformUtils.h"
#include "CoordinateTransform.h"

namespace
{
    constexpr CameraConfig kDefaultCamera{
        2.0,
        800.0,
        800.0,
        640.0,
        360.0
    };

    constexpr TransformMatrix kIdentityTransform{ {
        {{1.0, 0.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0, 0.0}},
        {{0.0, 0.0, 1.0, 0.0}},
        {{0.0, 0.0, 0.0, 1.0}}
    } };

    constexpr TransformMatrix kInvalidTransform{ {
        {{0.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0, 1.0}}
    } };

    struct InvalidCameraCase
    {
        const char* name;
        CameraConfig config;
    };

    struct InvalidTransformCase
    {
        const char* name;
        TransformMatrix transform;
    };

    struct InvalidProjectionCase
    {
        const char* name;
        Target target;
        CameraConfig config;
    };

    void PrintTo(
        const InvalidCameraCase& testCase,
        std::ostream* output)
    {
        *output << testCase.name;
    }

    void PrintTo(
        const InvalidTransformCase& testCase,
        std::ostream* output)
    {
        *output << testCase.name;
    }

    void PrintTo(
        const InvalidProjectionCase& testCase,
        std::ostream* output)
    {
        *output << testCase.name;
    }
}

//参数化夹具
class InvalidCameraConfigTest:
    public ::testing::TestWithParam<InvalidCameraCase>{};

class InvalidTransformMatrixTest :
    public ::testing::TestWithParam<InvalidTransformCase>{};

class InvalidProjectionParametersTest :
    public ::testing::TestWithParam<InvalidProjectionCase>{};

//TEST_P 同一个测试逻辑，自动跑多组不同输入
TEST_P(
    InvalidCameraConfigTest,
    RejectsAndPreservesStatus)
{
    // GetParam() 从TestWithParam基类拿当前这一组参数
    const InvalidCameraCase& testCase = GetParam();
    //下面的全是复用的测试代码
    RobotVision vision(testCase.config, kIdentityTransform);

    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::InvalidCameraConfig);

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

    EXPECT_FALSE(result.has_value());

    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::InvalidCameraConfig);
}

//定义所有参数公用的测试逻辑
INSTANTIATE_TEST_SUITE_P(
    InvalidCameraValues,  //测试套件前缀名(在报告里显示)
    InvalidCameraConfigTest, //对应上面的TestWithParam夹具类
    ::testing::Values(
        InvalidCameraCase{
            "ZeroDepth",
            {
                0.0,
                800.0,
                800.0,
                640.0,
                360.0
            }
        },
        InvalidCameraCase{
            "ZeroFx",
            {
                2.0,
                0.0,
                800.0,
                640.0,
                360.0
            }
        },
        InvalidCameraCase{
            "ZeroFy",
            {
                2.0,
                800.0,
                0.0,
                640.0,
                360.0
            }
        },
        InvalidCameraCase{
            "NaNDepth",
            {
                std::numeric_limits<double>::quiet_NaN(),
                800.0,
                800.0,
                640.0,
                360.0
            }
        },
        InvalidCameraCase{
            "InfiniteFx",
            {
                2.0,
                std::numeric_limits<double>::infinity(),
                800.0,
                640.0,
                360.0
            }
        },
        InvalidCameraCase{
            "NaNCx",
            {
                2.0,
                800.0,
                800.0,
                std::numeric_limits<double>::quiet_NaN(),
                360.0
            }
        }
    ),
    [](
        const ::testing::TestParamInfo<
        InvalidCameraCase>& info)
    {
        return std::string(
            info.param.name);
    }
);

INSTANTIATE_TEST_SUITE_P(
    InvalidTransformValues,
    InvalidTransformMatrixTest,
    ::testing::Values(
        InvalidTransformCase{
            "InvalidHomogeneousRow",
            TransformMatrix{{
                {{1.0, 0.0, 0.0, 0.7}},
                {{0.0, 1.0, 0.0, 2.1}},
                {{0.0, 0.0, 1.0, 3.0}},
                {{0.0, 0.0, 0.0, 2.0}}
            }}
        },
        InvalidTransformCase{
            "InfiniteTranslation",
            TransformMatrix{{
                {{
                    1.0,
                    0.0,
                    0.0,
                    std::numeric_limits<double>::infinity()
                }},
                {{0.0, 1.0, 0.0, 0.0}},
                {{0.0, 0.0, 1.0, 0.0}},
                {{0.0, 0.0, 0.0, 1.0}}
            }}
        }
    ),
    [](
        const ::testing::TestParamInfo<
        InvalidTransformCase>& info)
    {
        return std::string(
            info.param.name);
    }
);

INSTANTIATE_TEST_SUITE_P(
    InvalidProjectionValues,
    InvalidProjectionParametersTest,
    ::testing::Values(
        InvalidProjectionCase{
            "ZeroDepth",
            {
                1,
                0.90,
                720.0,
                400.0,
                false
            },
            {
                0.0,
                800.0,
                800.0,
                640.0,
                360.0
            }
        },
        InvalidProjectionCase{
            "ZeroFx",
            {
                1,
                0.90,
                720.0,
                400.0,
                false
            },
            {
                2.0,
                0.0,
                800.0,
                640.0,
                360.0
            }
        },
        InvalidProjectionCase{
            "ZeroFy",
            {
                1,
                0.90,
                720.0,
                400.0,
                false
            },
            {
                2.0,
                800.0,
                0.0,
                640.0,
                360.0
            }
        },
        InvalidProjectionCase{
            "InfiniteFx",
            {
                1,
                0.90,
                720.0,
                400.0,
                false
            },
            {
                2.0,
                std::numeric_limits<double>::infinity(),
                800.0,
                640.0,
                360.0
            }
        },
        InvalidProjectionCase{
            "NaNTargetX",
            {
                2,
                0.90,
                std::numeric_limits<double>::quiet_NaN(),
                400.0,
                false
            },
            {
                2.0,
                800.0,
                800.0,
                640.0,
                360.0
            }
        },
        InvalidProjectionCase{
            "InfiniteTargetY",
            {
                3,
                0.90,
                720.0,
                std::numeric_limits<double>::infinity(),
                false
            },
            {
                2.0,
                800.0,
                800.0,
                640.0,
                360.0
            }
        }
    ),
    [](
        const ::testing::TestParamInfo<
        InvalidProjectionCase>& info)
    {
        return std::string(
            info.param.name);
    }
);

TEST(
    RobotVisionValidationTest,
    InvalidTransformIsPreserved)
{
    // 使用合法相机擦参数，只制造变换矩阵错误
    RobotVision vision(kDefaultCamera, kInvalidTransform);

    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::InvalidTransform);

    const std::vector<Target> targets{
        {1, 0.90, 720.0, 400.0, false}
    };

    const std::optional<ValidTarget> result = vision.run(targets);

    EXPECT_FALSE(result.has_value());

    EXPECT_EQ(vision.getStatus(), VisionStatus::InvalidTransform);
}

TEST(
    TransformUtilsValidationTest,
    InvalidTransformHasNoInverse)
{
    const std::optional<TransformMatrix> inverse =
        TransformUtils::inverseTransform(
            kInvalidTransform);

    EXPECT_FALSE(
        inverse.has_value());
}

TEST_P(
    InvalidTransformMatrixTest,
    RejectsValidationAndInversion)
{
    const InvalidTransformCase& testCase =
        GetParam();

    EXPECT_FALSE(
        TransformUtils::isValidTransformMatrix(
            testCase.transform));

    const std::optional<TransformMatrix> inverse =
        TransformUtils::inverseTransform(
            testCase.transform);

    EXPECT_FALSE(
        inverse.has_value());
}

TEST_P(
    InvalidProjectionParametersTest,
    RejectsInvalidInput)
{
    const InvalidProjectionCase& testCase =
        GetParam();

    const std::optional<CameraPoint> result =
        CoordinateTransform::targetToCamera(
            testCase.target,
            testCase.config);

    EXPECT_FALSE(
        result.has_value());
}

TEST(CameraConfigValidationTest, RequiresBothImageDimensions)
{
    CameraConfig camera{
        2.0, 800.0, 800.0, 640.0, 360.0
    };

    // 旧配置：两个尺寸都是 0，暂时允许。
    EXPECT_TRUE(
        CoordinateTransform::isValidCameraConfig(camera));

    camera.imageWidth = 1920;
    EXPECT_FALSE(
        CoordinateTransform::isValidCameraConfig(camera));

    camera.imageHeight = 1080;
    EXPECT_TRUE(
        CoordinateTransform::isValidCameraConfig(camera));

    camera.imageWidth = -1;
    EXPECT_FALSE(
        CoordinateTransform::isValidCameraConfig(camera));
}

TEST(CameraConfigValidationTest, MatchesDeclaredImageSize)
{
    CameraConfig camera{
        2.0, 800.0, 800.0, 640.0, 360.0
    };

    EXPECT_FALSE(
        CoordinateTransform::matchesImageSize(
            camera, 1920, 1080));

    camera.imageWidth = 1920;
    camera.imageHeight = 1080;

    EXPECT_TRUE(
        CoordinateTransform::matchesImageSize(
            camera, 1920, 1080));

    EXPECT_FALSE(
        CoordinateTransform::matchesImageSize(
            camera, 1280, 720));
}

TEST(CameraConfigValidationTest, RejectsNonFiniteDistortion)
{
    CameraConfig camera{
        2.0, 800.0, 800.0, 640.0, 360.0
    };

    camera.distortionCoefficients[0] =
        std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(
        CoordinateTransform::isValidCameraConfig(camera));

    camera.distortionCoefficients[0] = 0.0;
    camera.distortionCoefficients[4] =
        std::numeric_limits<double>::infinity();

    EXPECT_FALSE(
        CoordinateTransform::isValidCameraConfig(camera));
}

TEST(FrameSyncTest, SelectsClosestDepthWithinThreshold)
{
    const std::vector<TimedDepthFrame> depths{
        {FrameTimestamp{94.0, ClockDomain::VideoTimeline}, {}},
        {FrameTimestamp{108.0, ClockDomain::VideoTimeline}, {}}
    };

    const TimedDepthFrame* result =
        findClosestDepthFrame(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            depths,
            10.0);

    ASSERT_NE(result, nullptr);
    EXPECT_EQ(result, &depths[0]);
}

TEST(FrameSyncTest, SkipsInvalidDepthTimestamp)
{
    const double nan =
        std::numeric_limits<double>::quiet_NaN();

    const std::vector<TimedDepthFrame> depths{
        {FrameTimestamp{nan, ClockDomain::VideoTimeline}, {}},
        {FrameTimestamp{94.0, ClockDomain::VideoTimeline}, {}}
    };

    const TimedDepthFrame* result =
        findClosestDepthFrame(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            depths,
            10.0);

    ASSERT_NE(result, nullptr);
    EXPECT_EQ(result, &depths[1]);
}

TEST(FrameSyncTest, ReturnsNullWhenAllDepthFramesExceedThreshold)
{
    const std::vector<TimedDepthFrame> depths{
        {FrameTimestamp{88.0, ClockDomain::VideoTimeline}, {}},
        {FrameTimestamp{118.0, ClockDomain::VideoTimeline}, {}}
    };

    const TimedDepthFrame* result =
        findClosestDepthFrame(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            depths,
            10.0);

    EXPECT_EQ(result, nullptr);
}

TEST(FrameSyncTest, RejectsDifferentClockDomains)
{
    const std::vector<TimedDepthFrame> depths{
        {FrameTimestamp{94.0, ClockDomain::LocalSteady}, {}}
    };

    const TimedDepthFrame* result =
        findClosestDepthFrame(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            depths,
            10.0);

    EXPECT_EQ(result, nullptr);
}

TEST(FrameSyncTest, RejectsDifferentImageSizes)
{
    const DepthFrame depth{ 2, 2, {1000, 1000, 1000, 1000}};

    EXPECT_FALSE(canSampleAtColorPixels(
        depth, 3, 2, true));
}

TEST(FrameSyncTest, RejectsDepthNotAlignedToColor)
{
    const DepthFrame depth{ 2, 2, {1000, 1000, 1000, 1000}};

    EXPECT_FALSE(canSampleAtColorPixels(
        depth, 2, 2, false));
}

TEST(FrameSyncTest, AcceptsAlignedDepthWithMatchingImageSize)
{
    const DepthFrame depth{ 2, 2, {1000, 2000, 3000, 4000} };

    EXPECT_TRUE(canSampleAtColorPixels(
        depth, 2, 2, true));
}

TEST(FrameSyncTest, SelectsClosestTransform)
{
    const std::vector<TimedTransform> transforms{
        {
            FrameTimestamp{94.0, ClockDomain::VideoTimeline},
            TransformMatrix{}
        },
        {
            FrameTimestamp{108.0, ClockDomain::VideoTimeline},
            TransformMatrix{}
        },
        {
            FrameTimestamp{130.0, ClockDomain::VideoTimeline},
            TransformMatrix{}
        }
    };

    const TimedTransform* result =
        findClosestTransform(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            transforms,
            10.0);

    ASSERT_NE(result, nullptr);  // 断言a!= b
    EXPECT_EQ(result, &transforms[0]);
}

TEST(FrameSyncTest, RejectsTransformOutsideTimeWindow)
{
    const std::vector<TimedTransform> transforms{
        {
            FrameTimestamp{130.0, ClockDomain::VideoTimeline},
            TransformMatrix{}
        }
    };

    const TimedTransform* result =
        findClosestTransform(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            transforms,
            10.0);

    EXPECT_EQ(result, nullptr);
}

TEST(FrameSyncTest, UsesSelectedTransformForRobotCoordinates)
{
    TransformMatrix transform94 = kIdentityTransform;
    transform94[0][3] = 0.7;

    TransformMatrix transform108 = kIdentityTransform;
    transform108[0][3] = 1.0;

    const std::vector<TimedTransform> transforms{
        {
            FrameTimestamp{94.0, ClockDomain::VideoTimeline},
            transform94
        },
        {
            FrameTimestamp{108.0, ClockDomain::VideoTimeline},
            transform108
        }
    };

    const TimedTransform* matched =
        findClosestTransform(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            transforms,
            10.0);

    ASSERT_NE(matched, nullptr);
    ASSERT_EQ(matched, &transforms[0]);

    const CameraPoint cameraPoint{ 0.2, 0.1, 2.0 };

    const RobotPoint robotPoint =
        CoordinateTransform::cameraToRobot(
            cameraPoint,
            matched->cameraToRobot);

    EXPECT_NEAR(robotPoint.X, 0.9, 1e-12);
    EXPECT_NEAR(robotPoint.Y, 0.1, 1e-12);
    EXPECT_NEAR(robotPoint.Z, 2.0, 1e-12);
}

TEST(RobotVisionTest, UsesFrameTransformWithoutChangingFixedTransform)
{
    const CameraConfig camera{
        2.0, 2.0, 2.0, 0.0, 0.0
    };

    const std::vector<Target> targets{
        {1, 0.90, 0.2, 0.1, false}
    };

    RobotVision vision(camera, kIdentityTransform);

    TransformMatrix firstTransform = kIdentityTransform;
    firstTransform[0][3] = 0.7;

    TransformMatrix secondTransform = kIdentityTransform;
    secondTransform[0][3] = 1.0;

    const auto first =
        vision.runWithTransform(targets, firstTransform);

    ASSERT_TRUE(first.has_value());
    EXPECT_NEAR(first->robotPoint.X, 0.9, 1e-12);
    EXPECT_NEAR(first->robotPoint.Y, 0.1, 1e-12);
    EXPECT_NEAR(first->robotPoint.Z, 2.0, 1e-12);

    const auto second =
        vision.runWithTransform(targets, secondTransform);

    ASSERT_TRUE(second.has_value());
    EXPECT_NEAR(second->robotPoint.X, 1.2, 1e-12);
    EXPECT_EQ(vision.getStatus(), VisionStatus::OK);

    const auto fixed = vision.run(targets);

    ASSERT_TRUE(fixed.has_value());
    EXPECT_NEAR(fixed->robotPoint.X, 0.2, 1e-12);
}

TEST(RobotVisionTest, RecoversAfterInvalidFrameTransform)
{
    const std::vector<Target> targets{
        {1, 0.90, 640.0, 360.0, false}
    };

    RobotVision vision(kDefaultCamera, kIdentityTransform);

    const auto rejected =
        vision.runWithTransform(targets, kInvalidTransform);

    EXPECT_FALSE(rejected.has_value());
    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::InvalidFrameTransform);

    const auto recovered =
        vision.runWithTransform(targets, kIdentityTransform);

    ASSERT_TRUE(recovered.has_value());
    EXPECT_EQ(vision.getStatus(), VisionStatus::OK);
    EXPECT_NEAR(recovered->robotPoint.X, 0.0, 1e-12);
    EXPECT_NEAR(recovered->robotPoint.Y, 0.0, 1e-12);
    EXPECT_NEAR(recovered->robotPoint.Z, 2.0, 1e-12);
}
