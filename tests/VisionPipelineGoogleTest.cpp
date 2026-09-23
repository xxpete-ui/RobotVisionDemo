#include "MockDetector.h"
#include "RobotVision.h"
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