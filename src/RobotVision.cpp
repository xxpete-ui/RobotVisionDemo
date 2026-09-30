#include "RobotVision.h"
#include "Logger.h"
#include "TargetProcessing.h"
#include "CoordinateTransform.h"
#include "TransformUtils.h"
#include <optional>


std::vector<ValidTarget> RobotVision::processTargets(
    const std::vector<Target>& targets,
    const TransformMatrix& transform) const
{
    std::vector<ValidTarget> validTargets;
    validTargets.reserve(targets.size());
    // 相机坐标{X, Y, Z}
    
    for (const auto& target : targets)
    {
        if (target.grabbed)
        {
            continue;
        }
        // 作用域 cameraPoint 只在花括号内可见
        const std::optional<CameraPoint> cameraPoint =
            CoordinateTransform::targetToCamera(
                target,
                cameraConfig);

        if (!cameraPoint)
        {
            Logger::warn("无效目标，跳过");
            continue;
        }

        const RobotPoint robotPoint =
            CoordinateTransform::cameraToRobot(
                *cameraPoint,
                transform);

        validTargets.push_back({
            target,
            *cameraPoint,
            robotPoint
            });
    }
    return validTargets;

}

std::optional<ValidTarget>RobotVision::runVisionPipeline(
    const std::vector<Target>& targets,
    const TransformMatrix& transform) const
{
    const std::vector<ValidTarget> validTargets =
        processTargets(targets, transform);

    const ValidTarget* selectedTarget =
        TargetProcessing::selectBestValidTarget(
            validTargets);

    if (selectedTarget == nullptr)
    {
        return std::nullopt;
    }

    return *selectedTarget;
}

RobotVision::RobotVision(
    const CameraConfig& config,
    const TransformMatrix& transform)
    : cameraConfig(config),
    T(transform),
    status(VisionStatus::OK)
{
    if (!checkCameraConfig())
    {
        status = VisionStatus::InvalidCameraConfig;
    }
    else if (!checkTransform())
    {
        status = VisionStatus::InvalidTransform;
    }
}

std::optional<ValidTarget> RobotVision::run(
    const std::vector<Target>& targets)
{
    if (status == VisionStatus::InvalidCameraConfig ||
        status == VisionStatus::InvalidTransform)
    {
        Logger::warn("RobotVision 状态异常");
        return std::nullopt;
    }

    const std::optional<ValidTarget> selectedTarget =
        runVisionPipeline(targets, T);

    if (!selectedTarget)
    {
        status = VisionStatus::NoValidTarget;
        Logger::warn("没有有效目标，跳过");
        return std::nullopt;
    }

    status = VisionStatus::OK;
    return selectedTarget;
}

bool RobotVision::checkCameraConfig() const
{
    return CoordinateTransform::isValidCameraConfig(
        cameraConfig);
}

bool RobotVision::checkTransform() const
{
    return TransformUtils::isValidTransformMatrix(T);
}

VisionStatus RobotVision::getStatus() const
{
    return status;
}

std::optional<ValidTarget> RobotVision::runWithTransform(
    const std::vector<Target>& targets,
    const TransformMatrix& transform)
{
    // Preserve permanent initialization errors.
    if (status == VisionStatus::InvalidCameraConfig ||
        status == VisionStatus::InvalidTransform)
    {
        return std::nullopt;
    }

    // Reject this frame, but allow a later valid frame to recover.
    if (!TransformUtils::isValidTransformMatrix(transform))
    {
        status = VisionStatus::InvalidFrameTransform;
        return std::nullopt;
    }

    const std::optional<ValidTarget> selectedTarget =
        runVisionPipeline(targets, transform);

    if (!selectedTarget)
    {
        status = VisionStatus::NoValidTarget;
        return std::nullopt;
    }

    status = VisionStatus::OK;
    return selectedTarget;
}