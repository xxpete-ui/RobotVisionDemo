#pragma once

#include "VisionTypes.h"
#include "FrameSync.h"
#include <optional>

#include <vector>

class IDetector;
class RobotVision;
struct DepthFrame;

enum class DepthFusionStatus
{
    OK,              // 时间和空间检查通过，产生三维目标
    NoMatchedDepth,  // 没有符合时间条件的深度帧
    DepthNotAligned, // 选中了深度帧，但不能按彩色图像素坐标采样
    VisionProcessingFailed,    // 前两关通过，检测或深度采样后仍没有有效目标
    NoMatchedTransform
};

struct DepthFusionResult
{
    DepthFusionStatus status;
    std::optional<ValidTarget> target;
};

class VisionPipeline
{
public:
    VisionPipeline(
        IDetector& detector,
        RobotVision& robotVision);

    std::optional<ValidTarget> run();

    std::optional<ValidTarget> runWithDepth(
        const DepthFrame& depthFrame,
        int colorWidth,
        int colorHeight,
        bool depthAlignedToColor);

    DepthFusionResult runWithSyncedDepth(
        FrameTimestamp colorTimestamp,
        const std::vector<TimedDepthFrame>& depthFrames,
        double maxDeltaMs,
        int colorWidth,
        int colorHeight,
        bool depthAlignedToColor);

    DepthFusionResult runWithSyncedDepthAndTransform(
        FrameTimestamp colorTimestamp,
        const std::vector<TimedDepthFrame>& depthFrames,
        double maxDepthDeltaMs,
        const std::vector<TimedTransform>& transforms,
        double maxTransformDeltaMs,
        int colorWidth,
        int colorHeight,
        bool depthAlignedToColor);

private:
    IDetector& detector_;
    RobotVision& robotVision_;
};