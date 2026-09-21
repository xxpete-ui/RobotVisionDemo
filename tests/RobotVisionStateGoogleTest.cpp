#include "RobotVision.h"

#include <gtest/gtest.h>
#include "TargetProcessing.h"
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

    constexpr TransformMatrix kIdentityTransform{ {
        {{1.0, 0.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0, 0.0}},
        {{0.0, 0.0, 1.0, 0.0}},
        {{0.0, 0.0, 0.0, 1.0}}
    } };
}

class RobotVisionStateFixture :
    public ::testing::Test
{
protected:
    RobotVision vision{
        kDefaultCamera,
        kIdentityTransform
    };
};

// 测试空目标会产生NoValidTarget状态
TEST_F(
    RobotVisionStateFixture,
    EmptyTargetsSetNoValidTarget)
{
    const std::vector<Target> emptyTargets;

    const std::optional<ValidTarget> result =
        vision.run(emptyTargets);

    // 预期没有返回有效目标
    EXPECT_FALSE(result.has_value());

    // 预期状态变为NoValidTarget
    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::NoValidTarget);
}

// 测试NoValidTarget状态可以在下一帧恢复
TEST_F(
    RobotVisionStateFixture,
    RecoversAfterEmptyTargets)
{
    // 第一帧为空
    const std::vector<Target> emptyTargets;

    const std::optional<ValidTarget> emptyResult =
        vision.run(emptyTargets);

    EXPECT_FALSE(emptyResult.has_value());

    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::NoValidTarget);

    // 第二帧检测到有效目标
    const std::vector<Target> validTargets{
        {
            7,
            0.85,
            720.0,
            400.0,
            false
        }
    };

    const std::optional<ValidTarget> recoveredResult =
        vision.run(validTargets);

    // 后面的代码需要访问recoveredResult，因此使用ASSERT
    ASSERT_TRUE(recoveredResult.has_value());

    EXPECT_EQ(
        recoveredResult->target.id,
        7);

    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::OK);
}

TEST_F(
    RobotVisionStateFixture,
    RecoversWhenGrabbedTargetBecomesAvailable)
{
    std::vector<Target> targets{
        {
            3,
            0.80,
            720.0,
            400.0,
            true
        }
    };

    // 第一帧中，唯一目标已经被抓取，应当被过滤
    const std::optional<ValidTarget> unavailableResult =
        vision.run(targets);

    EXPECT_FALSE(
        unavailableResult.has_value());

    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::NoValidTarget);

    // 下一帧目标重新变为可用
    targets[0].grabbed = false;

    const std::optional<ValidTarget> recoveredResult =
        vision.run(targets);

    ASSERT_TRUE(
        recoveredResult.has_value());

    EXPECT_EQ(
        recoveredResult->target.id,
        3);

    EXPECT_FALSE(
        recoveredResult->target.grabbed);

    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::OK);
}

TEST_F(
    RobotVisionStateFixture,
    SelectsNextTargetAfterBestIsGrabbed)
{
    std::vector<Target> targets{
        {
            1,
            0.72,
            700.0,
            380.0,
            false
        },
        {
            2,
            0.75,
            720.0,
            400.0,
            false
        }
    };

    // 不存在的ID不能改变任何目标
    EXPECT_FALSE(
        TargetProcessing::markTargetGrabbed(
            targets,
            999));

    EXPECT_FALSE(targets[0].grabbed);
    EXPECT_FALSE(targets[1].grabbed);

    // 首次应选择置信度更高的目标2
    const std::optional<ValidTarget> firstResult =
        vision.run(targets);

    ASSERT_TRUE(firstResult.has_value());

    EXPECT_EQ(
        firstResult->target.id,
        2);

    // 将目标2标记为已抓取
    EXPECT_TRUE(
        TargetProcessing::markTargetGrabbed(
            targets,
            firstResult->target.id));

    EXPECT_TRUE(targets[1].grabbed);

    // 再次运行后应跳过目标2，选择目标1
    const std::optional<ValidTarget> secondResult =
        vision.run(targets);

    ASSERT_TRUE(secondResult.has_value());

    EXPECT_EQ(
        secondResult->target.id,
        1);

    EXPECT_EQ(
        vision.getStatus(),
        VisionStatus::OK);
}

TEST(
    TargetProcessingTest,
    HandlesEmptyAndEqualConfidenceTargets)
{
    const std::vector<Target> emptyTargets;

    EXPECT_EQ(
        TargetProcessing::selectBestTarget(
            emptyTargets),
        nullptr);

    const std::vector<Target> equalConfidenceTargets{
        {
            1,
            0.80,
            700.0,
            380.0,
            false
        },
        {
            2,
            0.80,
            720.0,
            400.0,
            false
        }
    };

    const Target* selectedTarget =
        TargetProcessing::selectBestTarget(
            equalConfidenceTargets);
    // 断言A != B
    ASSERT_NE(
        selectedTarget,
        nullptr);

    // 相同置信度时保留第一个目标
    EXPECT_EQ(
        selectedTarget,
        &equalConfidenceTargets.front());

    EXPECT_EQ(
        selectedTarget->id,
        1);

    const std::vector<ValidTarget> emptyValidTargets;

    EXPECT_EQ(
        TargetProcessing::selectBestValidTarget(
            emptyValidTargets),
        nullptr);
}