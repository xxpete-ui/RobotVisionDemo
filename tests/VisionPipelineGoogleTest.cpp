#include "MockDetector.h"
#include "RobotVision.h"
#include "DepthSampler.h"
#include "VisionPipeline.h"
#include "TargetProcessing.h"
#include <gtest/gtest.h>
#include <utility>
#include <optional>
#include <vector>


namespace {
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
}

TEST(
    VisionPipelineTest,
    SelectBestDetectorTarget)
{
    const std::vector<Target> detectedTargets{
        {1, 0.70, 680.0, 360.0 ,false},
        {2, 0.90, 720.0, 400.0, false}
    };

    MockDetector detector(detectedTargets);

    RobotVision robotVision(kDefaultCamera, kIdentityTransform);

    VisionPipeline pipeline(detector, robotVision);

    const std::optional<ValidTarget> result = pipeline.run();

    ASSERT_TRUE(result.has_value());

    EXPECT_EQ(result->target.id, 2);
    EXPECT_DOUBLE_EQ(result->target.confidence, 0.90);

    EXPECT_DOUBLE_EQ(result->cameraPoint.X, 0.2);
    EXPECT_DOUBLE_EQ(result->cameraPoint.Y, 0.1);
    EXPECT_DOUBLE_EQ(result->cameraPoint.Z, 2.0);

    EXPECT_DOUBLE_EQ(result->robotPoint.X, 0.2);
    EXPECT_DOUBLE_EQ(result->robotPoint.Y, 0.1);
    EXPECT_DOUBLE_EQ(result->robotPoint.Z, 2.0);

    EXPECT_EQ(robotVision.getStatus(), VisionStatus::OK);
}

TEST(
    VisionPipelineTest,
    PreservesClassAndOriginalBoxOfSelectedTarget)
{
    // 两辆模拟车辆。框和中心都使用原图像素坐标。
    // 第二辆置信度更高，应成为最终选中的目标。
    const std::vector<Target> detectedTargets{
        {1, 0.70, 680.0, 360.0, false,
         2, {650.0, 330.0, 60.0, 60.0}},

        {2, 0.90, 720.0, 400.0, false,
         2, {690.0, 370.0, 60.0, 60.0}}
    };

    MockDetector detector(detectedTargets);
    RobotVision robotVision(kDefaultCamera, kIdentityTransform);
    VisionPipeline pipeline(detector, robotVision);

    const std::optional<ValidTarget> result = pipeline.run();

    // ASSERT：没有结果时立刻停止本测试，避免后面解引用空 optional。
    ASSERT_TRUE(result.has_value());

    // 先确认选中的是第二辆，再检查它携带的信息。
    EXPECT_EQ(result->target.id, 2);
    EXPECT_EQ(result->target.classId, 2);

    EXPECT_DOUBLE_EQ(result->target.box.x, 690.0);
    EXPECT_DOUBLE_EQ(result->target.box.y, 370.0);
    EXPECT_DOUBLE_EQ(result->target.box.width, 60.0);
    EXPECT_DOUBLE_EQ(result->target.box.height, 60.0);

    EXPECT_DOUBLE_EQ(result->target.x, 720.0);
    EXPECT_DOUBLE_EQ(result->target.y, 400.0);
    EXPECT_EQ(robotVision.getStatus(), VisionStatus::OK);
}

TEST(
    VisionPipelineTest,
    EmptyDetectionSetsNoValidTarget)
{
    MockDetector detector({});

    RobotVision robotVision(
        kDefaultCamera,
        kIdentityTransform);

    VisionPipeline pipeline(
        detector,
        robotVision);

    const std::optional<ValidTarget> result =
        pipeline.run();

    EXPECT_FALSE(result.has_value());

    EXPECT_EQ(
        robotVision.getStatus(),
        VisionStatus::NoValidTarget);
}

TEST(
    VisionPipelineTest,
    HandlesChangingTargetsAcrossFrames)
{
    MockDetector detector({});

    RobotVision robotVision(
        kDefaultCamera,
        kIdentityTransform);

    VisionPipeline pipeline(
        detector,
        robotVision);

    // 第一帧：没有检测到目标
    const std::optional<ValidTarget> firstResult =
        pipeline.run();

    EXPECT_FALSE(firstResult.has_value());

    EXPECT_EQ(
        robotVision.getStatus(),
        VisionStatus::NoValidTarget);

    // 第二帧：检测到两个未抓取目标
    detector.setTargets({
        {1, 0.70, 680.0, 360.0, false},
        {2, 0.90, 720.0, 400.0, false}
        });

    const std::optional<ValidTarget> secondResult =
        pipeline.run();

    ASSERT_TRUE(secondResult.has_value());

    EXPECT_EQ(secondResult->target.id, 2);

    EXPECT_EQ(
        robotVision.getStatus(),
        VisionStatus::OK);

    // 第三帧：目标 2 已经抓取
    std::vector<Target> thirdFrameTargets =
        detector.detect();

    const bool marked =
        TargetProcessing::markTargetGrabbed(
            thirdFrameTargets,
            secondResult->target.id);

    ASSERT_TRUE(marked);

    detector.setTargets(
        std::move(thirdFrameTargets));

    const std::optional<ValidTarget> thirdResult =
        pipeline.run();

    ASSERT_TRUE(thirdResult.has_value());

    EXPECT_EQ(thirdResult->target.id, 1);

    EXPECT_EQ(
        robotVision.getStatus(),
        VisionStatus::OK);
}

TEST(
    VisionPipelineTest,
    RunWithDepthSkipsTargetWithoutValidDepth)
{
    // 第 1 行第 1 列为 0：没有有效深度；
    // 第 0 行第 1 列为 2000 mm：有效深度 2 m。
    const DepthFrame depthFrame{
        3,
        2,
        {1000, 2000, 3000,
         4000,    0, 6000}
    };

    MockDetector detector({
        {1, 0.99, 3.0, 1.0, false}, // 置信度高，深度无效
        {2, 0.80, 1.0, 0.0, false}  // 置信度低，深度有效
        });

    // 与这张 3×2 测试图配套的简单内参。
    const CameraConfig camera{
        2.0, 2.0, 2.0, 0.0, 0.0
    };

    RobotVision vision(camera, kIdentityTransform);
    VisionPipeline pipeline(detector, vision);

    const std::optional<ValidTarget> result =
        pipeline.runWithDepth(depthFrame, 3, 2, true);

    ASSERT_TRUE(result.has_value());

    // ID 1 在深度采样阶段被排除；只能选 ID 2。
    EXPECT_EQ(result->target.id, 2);
    ASSERT_TRUE(result->target.depthMeters.has_value());
    EXPECT_DOUBLE_EQ(*result->target.depthMeters, 2.0);

    // 像素 (1,0)、深度 2m、fx=2、cx=0；
    // 单位变换矩阵下，机器人点与相机点相同。
    EXPECT_DOUBLE_EQ(result->cameraPoint.X, 1.0);
    EXPECT_DOUBLE_EQ(result->cameraPoint.Y, 0.0);
    EXPECT_DOUBLE_EQ(result->cameraPoint.Z, 2.0);
    EXPECT_DOUBLE_EQ(result->robotPoint.X, 1.0);
    EXPECT_EQ(vision.getStatus(), VisionStatus::OK);
}

TEST(
    VisionPipelineTest,
    RunWithDepthReturnsNoTargetWhenDepthIsMissing)
{
    const DepthFrame depthFrame{
        3,
        2,
        {0, 0, 0,
         0, 0, 0}
    };

    // 中心 (1,1) 对应的深度是 0。
    MockDetector detector({
        {1, 0.99, 1.0, 1.0, false}
        });

    const CameraConfig camera{
        2.0, 2.0, 2.0, 0.0, 0.0
    };

    RobotVision vision(camera, kIdentityTransform);
    VisionPipeline pipeline(detector, vision);

    const std::optional<ValidTarget> result =
        pipeline.runWithDepth(depthFrame, 3, 2, true);

    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::NoValidTarget
    );
}

TEST(DepthSamplerTest, UsesNeighborMedianWhenCenterIsMissing)
{
    const DepthFrame frame{
        3,
        3,
        {1000, 2000, 3000,
         1000,    0, 3000,
         1000, 2000, 3000}
    };

    // 中心 (1,1) 为 0；周围八个有效值的中位数是 2000 mm。
    const auto depth =
        DepthSampler::sampleWithNeighborhood(
            frame, 1.0, 1.0);

    ASSERT_TRUE(depth.has_value());
    EXPECT_DOUBLE_EQ(*depth, 2.0);
}

TEST(DepthSamplerTest, AttachValidDepthRecoversCenterHole)
{
    const DepthFrame frame{
        3, 2,
        {1000, 2000, 3000,
         4000,    0, 6000}
    };

    const std::vector<Target> detected{
        {1, 0.90, 1.0, 1.0, false}
    };

    const auto result =
        DepthSampler::attachValidDepth(detected, frame);

    ASSERT_EQ(result.size(), 1U);
    ASSERT_TRUE(result[0].depthMeters.has_value());

    // 中心为 0；邻域五个有效值排序后为
    // 1000、2000、3000、4000、6000 mm，中位数为 3000 mm。
    EXPECT_DOUBLE_EQ(*result[0].depthMeters, 3.0);
}

TEST(VisionPipelineTest, RejectsDepthNotAlignedToColor)
{
    const DepthFrame depthFrame{
        2, 2,
        {2000, 2000,
         2000, 2000}
    };

    MockDetector detector({
        {1, 0.90, 1.0, 0.0, false}
        });

    const CameraConfig camera{
        2.0, 2.0, 2.0, 0.0, 0.0
    };

    RobotVision vision(camera, kIdentityTransform);
    VisionPipeline pipeline(detector, vision);

    const auto rejected =
        pipeline.runWithDepth(depthFrame, 2, 2, false);
    EXPECT_FALSE(rejected.has_value());

    const auto accepted =
        pipeline.runWithDepth(depthFrame, 2, 2, true);
    ASSERT_TRUE(accepted.has_value());
    ASSERT_TRUE(accepted->target.depthMeters.has_value());
    EXPECT_DOUBLE_EQ(*accepted->target.depthMeters, 2.0);
}

TEST(VisionPipelineTest, UsesClosestSynchronizedDepth)
{
    const std::vector<TimedDepthFrame> depths{
        {
            FrameTimestamp{94.0, ClockDomain::VideoTimeline},
            {2, 2, {2000, 2000, 2000, 2000}}
        },
        {
            FrameTimestamp{108.0, ClockDomain::VideoTimeline},
            {2, 2, {3000, 3000, 3000, 3000}}
        }
    };

    MockDetector detector({
        {1, 0.90, 1.0, 0.0, false}
        });

    const CameraConfig camera{
        2.0, 2.0, 2.0, 0.0, 0.0
    };

    RobotVision vision(camera, kIdentityTransform);
    VisionPipeline pipeline(detector, vision);

    const DepthFusionResult result =
        pipeline.runWithSyncedDepth(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            depths,
            10.0,
            2, 2, true);

    ASSERT_EQ(result.status, DepthFusionStatus::OK);
    ASSERT_TRUE(result.target.has_value());
    ASSERT_TRUE(result.target->target.depthMeters.has_value());
    EXPECT_DOUBLE_EQ(*result.target->target.depthMeters, 2.0);
    EXPECT_DOUBLE_EQ(result.target->cameraPoint.Z, 2.0);
}

TEST(VisionPipelineTest, DistinguishesTimeAndAlignmentRejection)
{
    const DepthFrame depth{
        2, 2,
        {2000, 2000,
         2000, 2000}
    };

    MockDetector detector({
        {1, 0.90, 1.0, 0.0, false}
        });

    const CameraConfig camera{
        2.0, 2.0, 2.0, 0.0, 0.0
    };

    RobotVision vision(camera, kIdentityTransform);
    VisionPipeline pipeline(detector, vision);

    const std::vector<TimedDepthFrame> wrongClock{
        {FrameTimestamp{94.0, ClockDomain::LocalSteady}, depth}
    };

    const DepthFusionResult timeRejected =
        pipeline.runWithSyncedDepth(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            wrongClock, 10.0, 2, 2, true);

    EXPECT_EQ(timeRejected.status, DepthFusionStatus::NoMatchedDepth);
    EXPECT_FALSE(timeRejected.target.has_value());

    const std::vector<TimedDepthFrame> matchingClock{
        {FrameTimestamp{94.0, ClockDomain::VideoTimeline}, depth}
    };

    const DepthFusionResult alignmentRejected =
        pipeline.runWithSyncedDepth(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            matchingClock, 10.0, 2, 2, false);

    EXPECT_EQ(
        alignmentRejected.status,
        DepthFusionStatus::DepthNotAligned);
    EXPECT_FALSE(alignmentRejected.target.has_value());
}

TEST(VisionPipelineTest, UsesSynchronizedDepthAndTransform)
{
    const std::vector<TimedDepthFrame> depths{
        {
            FrameTimestamp{94.0, ClockDomain::VideoTimeline},
            {2, 2, {2000, 2000, 2000, 2000}}
        },
        {
            FrameTimestamp{108.0, ClockDomain::VideoTimeline},
            {2, 2, {3000, 3000, 3000, 3000}}
        }
    };

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

    MockDetector detector({
        {1, 0.90, 1.0, 0.0, false}
        });

    const CameraConfig camera{
        5.0, 2.0, 2.0, 0.0, 0.0
    };

    RobotVision vision(camera, kIdentityTransform);
    VisionPipeline pipeline(detector, vision);

    const DepthFusionResult result =
        pipeline.runWithSyncedDepthAndTransform(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            depths,
            10.0,
            transforms,
            10.0,
            2, 2, true);

    ASSERT_EQ(result.status, DepthFusionStatus::OK);
    ASSERT_TRUE(result.target.has_value());
    ASSERT_TRUE(result.target->target.depthMeters.has_value());

    EXPECT_DOUBLE_EQ(*result.target->target.depthMeters, 2.0);

    EXPECT_NEAR(result.target->cameraPoint.X, 1.0, 1e-12);
    EXPECT_NEAR(result.target->cameraPoint.Y, 0.0, 1e-12);
    EXPECT_NEAR(result.target->cameraPoint.Z, 2.0, 1e-12);

    EXPECT_NEAR(result.target->robotPoint.X, 1.7, 1e-12);
    EXPECT_NEAR(result.target->robotPoint.Y, 0.0, 1e-12);
    EXPECT_NEAR(result.target->robotPoint.Z, 2.0, 1e-12);
}

TEST(VisionPipelineTest, RejectsTransformOutsideTimeWindow)
{
    const std::vector<TimedDepthFrame> depths{
        {
            FrameTimestamp{100.0, ClockDomain::VideoTimeline},
            {2, 2, {2000, 2000, 2000, 2000}}
        }
    };

    const std::vector<TimedTransform> transforms{
        {
            FrameTimestamp{130.0, ClockDomain::VideoTimeline},
            kIdentityTransform
        }
    };

    MockDetector detector({
        {1, 0.90, 1.0, 0.0, false}
        });

    const CameraConfig camera{
        2.0, 2.0, 2.0, 0.0, 0.0
    };

    RobotVision vision(camera, kIdentityTransform);
    VisionPipeline pipeline(detector, vision);

    const DepthFusionResult result =
        pipeline.runWithSyncedDepthAndTransform(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            depths,
            10.0,
            transforms,
            10.0,
            2, 2, true);

    EXPECT_EQ(result.status, DepthFusionStatus::NoMatchedTransform);
    EXPECT_FALSE(result.target.has_value());
}

TEST(VisionPipelineTest, RejectsTransformFromDifferentClockDomain)
{
    const std::vector<TimedDepthFrame> depths{
        {
            FrameTimestamp{100.0, ClockDomain::VideoTimeline},
            {2, 2, {2000, 2000, 2000, 2000}}
        }
    };

    const std::vector<TimedTransform> transforms{
        {
            FrameTimestamp{96.0, ClockDomain::LocalSteady},
            kIdentityTransform
        }
    };

    MockDetector detector({
        {1, 0.90, 1.0, 0.0, false}
        });

    const CameraConfig camera{
        2.0, 2.0, 2.0, 0.0, 0.0
    };

    RobotVision vision(camera, kIdentityTransform);
    VisionPipeline pipeline(detector, vision);

    const DepthFusionResult result =
        pipeline.runWithSyncedDepthAndTransform(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            depths,
            10.0,
            transforms,
            10.0,
            2, 2, true);

    EXPECT_EQ(
        result.status,
        DepthFusionStatus::NoMatchedTransform);
    EXPECT_FALSE(result.target.has_value());
}

TEST(VisionPipelineTest, RecoversAfterInvalidSynchronizedTransform)
{
    const std::vector<TimedDepthFrame> depths{
        {
            FrameTimestamp{100.0, ClockDomain::VideoTimeline},
            {2, 2, {2000, 2000, 2000, 2000}}
        }
    };

    TransformMatrix invalidTransform = kIdentityTransform;
    invalidTransform[0][0] = 2.0;

    const std::vector<TimedTransform> invalidTransforms{
        {
            FrameTimestamp{96.0, ClockDomain::VideoTimeline},
            invalidTransform
        }
    };

    MockDetector detector({
        {1, 0.90, 1.0, 0.0, false}
        });

    const CameraConfig camera{
        2.0, 2.0, 2.0, 0.0, 0.0
    };

    RobotVision vision(camera, kIdentityTransform);
    VisionPipeline pipeline(detector, vision);

    const DepthFusionResult rejected =
        pipeline.runWithSyncedDepthAndTransform(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            depths,
            10.0,
            invalidTransforms,
            10.0,
            2, 2, true);

    EXPECT_EQ(
        rejected.status,
        DepthFusionStatus::VisionProcessingFailed);
    EXPECT_FALSE(rejected.target.has_value());
    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::InvalidFrameTransform);

    TransformMatrix validTransform = kIdentityTransform;
    validTransform[0][3] = 0.7;

    const std::vector<TimedTransform> validTransforms{
        {
            FrameTimestamp{96.0, ClockDomain::VideoTimeline},
            validTransform
        }
    };

    const DepthFusionResult recovered =
        pipeline.runWithSyncedDepthAndTransform(
            FrameTimestamp{ 100.0, ClockDomain::VideoTimeline },
            depths,
            10.0,
            validTransforms,
            10.0,
            2, 2, true);

    ASSERT_EQ(recovered.status, DepthFusionStatus::OK);
    ASSERT_TRUE(recovered.target.has_value());
    EXPECT_EQ(vision.getStatus(), VisionStatus::OK);

    EXPECT_NEAR(recovered.target->robotPoint.X, 1.7, 1e-12);
    EXPECT_NEAR(recovered.target->robotPoint.Y, 0.0, 1e-12);
    EXPECT_NEAR(recovered.target->robotPoint.Z, 2.0, 1e-12);
}