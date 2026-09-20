#include "RobotVision.h"
#include "Logger.h"
#include "VisionTypes.h"
#include "TargetProcessing.h"
#include "CoordinateTransform.h"
#include "TransformUtils.h"
#include <cmath>
#include <iostream>


const char* RobotVision::statusToString(VisionStatus status) {
    switch (status) {
        case VisionStatus::OK:
            return "OK";

        case VisionStatus::InvalidCameraConfig:
            return "InvalidCameraConfig";

        case VisionStatus::InvalidTransform:
            return "InvalidTransform";

        case VisionStatus::NoValidTarget:
            return "NoValidTarget";

        default:
            return "Unknown";
    }
}


std::vector<ValidTarget> RobotVision::processTargets(
    const std::vector<Target>& targets) {
    std::vector<ValidTarget> validTargets;
    // 相机坐标{X, Y, Z}
    
    for (const auto& target : targets) {
        if (target.grabbed) {
            continue;
        }
        CameraPoint cameraPoint;
        bool cameraPoint_result = CoordinateTransform::targetToCamera(target,
            cameraConfig.Z,
            cameraConfig.fx,
            cameraConfig.fy,
            cameraConfig.cx,
            cameraConfig.cy,
            cameraPoint);
        if (!cameraPoint_result) {
            Logger::warn("无效目标，跳过");
            continue;
        }
        RobotPoint robotPoint = CoordinateTransform::cameraToRobot(cameraPoint, T);
        ValidTarget validTarget{};
        validTarget.target = target;
        validTarget.cameraPoint = cameraPoint;
        validTarget.robotPoint = robotPoint;
        validTargets.push_back(validTarget);
    }
    return validTargets;

}

bool RobotVision::runVisionPipeline(
    const std::vector<Target>& targets,
    ValidTarget& bestTarget
) {
    std::vector<ValidTarget> processPipeline_result = RobotVision::processTargets(targets);
    if (processPipeline_result.empty()) {
        status = VisionStatus::NoValidTarget;
        Logger::warn("没有有效目标，跳过");
        return false;
    }
    
    const ValidTarget* bestValidTarget = TargetProcessing::selectBestValidTarget(processPipeline_result);
    if (bestValidTarget == nullptr) {
        status = VisionStatus::NoValidTarget;
        Logger::warn("没有有效目标，跳过");
        return false;
    }

    bestTarget = *bestValidTarget;
    status = VisionStatus::OK;
    return true;
}

RobotVision::RobotVision(
    const CameraConfig& config,
    const TransformMatrix& transform)
    : cameraConfig(config),
      status(VisionStatus::OK)

{
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            this->T[row][col] = transform[row][col];
        }
    }

    if (!checkCameraConfig()) {
        status = VisionStatus::InvalidCameraConfig;
    }

    else if (!checkTransform()) {
        status = VisionStatus::InvalidTransform;
    }
}

bool RobotVision::run(
    const std::vector<Target>& targets,
    ValidTarget& bestTarget)
{
    if (status == VisionStatus::InvalidCameraConfig ||
        status == VisionStatus::InvalidTransform) {
        Logger::warn("RobotVision 状态异常");
        return false;
    }
    // 新的一帧开始，清除上一帧的 NoValidTarget
    status = VisionStatus::OK;
    return runVisionPipeline(targets, bestTarget);
}

bool RobotVision::checkCameraConfig() {
    return cameraConfig.Z > 0 && cameraConfig.fx > 0 && cameraConfig.fy > 0;
}

bool RobotVision::checkTransform()
{
    return TransformUtils::isValidTransformMatrix(T);
}

VisionStatus RobotVision::getStatus() const
{
    return status;
}