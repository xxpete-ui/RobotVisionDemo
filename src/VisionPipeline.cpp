#include "VisionPipeline.h"
#include "FrameSync.h"
#include "IDetector.h"
#include "RobotVision.h"
#include "DepthSampler.h"

#include <vector>

VisionPipeline::VisionPipeline(
    IDetector& detector,
    RobotVision& robotVision)
    : detector_(detector),
    robotVision_(robotVision)
{
}

std::optional<ValidTarget> VisionPipeline::run()
{
    const std::vector<Target> targets =
        detector_.detect();

    return robotVision_.run(targets);
}

std::optional<ValidTarget> VisionPipeline::runWithDepth(
    const DepthFrame& depthFrame,
    int colorWidth,
    int colorHeight,
    bool depthAlignedToColor)
{
    if (!canSampleAtColorPixels(
        depthFrame,
        colorWidth,
        colorHeight,
        depthAlignedToColor))
    {
        return std::nullopt;
    }

    // 1. 检测器产生这一帧的全部目标。
    const std::vector<Target> detectedTargets =
        detector_.detect();

    // 2. 按每个目标的像素中心采样深度。
    //    无效深度目标在这里被移除，不会回退到演示用的 config.Z。
    const std::vector<Target> targetsWithDepth =
        DepthSampler::attachValidDepth(
            detectedTargets,
            depthFrame);

    // 3. 对留下的目标计算相机/机器人坐标并选择最佳目标。
    return robotVision_.run(targetsWithDepth);
}

DepthFusionResult VisionPipeline::runWithSyncedDepth(
    FrameTimestamp colorTimestamp,
    const std::vector<TimedDepthFrame>& depthFrames,
    double maxDeltaMs,
    int colorWidth,
    int colorHeight,
    bool depthAlignedToColor)
{
    {
        // 时间配对
        const TimedDepthFrame* matchedDepth =
            findClosestDepthFrame(
                colorTimestamp,
                depthFrames,
                maxDeltaMs);

        if (matchedDepth == nullptr)
        {
            return { DepthFusionStatus::NoMatchedDepth, std::nullopt };
        }

        // 空间检查
        if (!canSampleAtColorPixels(
            matchedDepth->frame,
            colorWidth,
            colorHeight,
            depthAlignedToColor))
        {
            return { DepthFusionStatus::DepthNotAligned, std::nullopt };
        }

        // 原有检测与三维计算
        const std::optional<ValidTarget> target =
            runWithDepth(
                matchedDepth->frame,
                colorWidth,
                colorHeight,
                depthAlignedToColor);

        if (!target)
        {
            return { DepthFusionStatus::VisionProcessingFailed, std::nullopt };
        }

        return { DepthFusionStatus::OK, target };
    }
}

DepthFusionResult VisionPipeline::runWithSyncedDepthAndTransform(
    FrameTimestamp colorTimestamp,
    const std::vector<TimedDepthFrame>& depthFrames,
    double maxDepthDeltaMs,
    const std::vector<TimedTransform>& transforms,
    double maxTransformDeltaMs,
    int colorWidth,
    int colorHeight,
    bool depthAlignedToColor)
{
    // 1. Match depth to the color frame.
    const TimedDepthFrame* matchedDepth =
        findClosestDepthFrame(
            colorTimestamp,
            depthFrames,
            maxDepthDeltaMs);

    if (matchedDepth == nullptr)
    {
        return { DepthFusionStatus::NoMatchedDepth, std::nullopt };
    }

    // 2. Check whether color pixels can index this depth frame.
    if (!canSampleAtColorPixels(
        matchedDepth->frame,
        colorWidth,
        colorHeight,
        depthAlignedToColor))
    {
        return { DepthFusionStatus::DepthNotAligned, std::nullopt };
    }

    // 3. Match the transform to the same color frame.
    const TimedTransform* matchedTransform =
        findClosestTransform(
            colorTimestamp,
            transforms,
            maxTransformDeltaMs);

    if (matchedTransform == nullptr)
    {
        return { DepthFusionStatus::NoMatchedTransform, std::nullopt };
    }

    // 4. Detect targets and attach sampled depth.
    const std::vector<Target> detectedTargets =
        detector_.detect();

    const std::vector<Target> targetsWithDepth =
        DepthSampler::attachValidDepth(
            detectedTargets,
            matchedDepth->frame);

    // 5. Calculate coordinates using the selected transform.
    const std::optional<ValidTarget> target =
        robotVision_.runWithTransform(
            targetsWithDepth,
            matchedTransform->cameraToRobot);

    if (!target)
    {
        return {
            DepthFusionStatus::VisionProcessingFailed,
            std::nullopt
        };
    }

    return { DepthFusionStatus::OK, target };
}