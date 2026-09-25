#include "VisionPipeline.h"

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
    const DepthFrame& depthFrame)
{
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
