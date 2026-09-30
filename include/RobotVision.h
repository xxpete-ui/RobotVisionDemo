#pragma once
#include "VisionTypes.h"
#include <vector>
#include <optional>

class RobotVision {
public:

    RobotVision(
        const CameraConfig& cameraConfig,
        const TransformMatrix& transform);

    [[nodiscard]]
    std::optional<ValidTarget> run(
        const std::vector<Target>& targets);

    [[nodiscard]]
    VisionStatus getStatus() const;

    [[nodiscard]]
    std::optional<ValidTarget> runWithTransform(
        const std::vector<Target>& targets,
        const TransformMatrix& transform);

private:
    CameraConfig cameraConfig;
    TransformMatrix T{};

    VisionStatus status;
    bool checkCameraConfig() const;
    
    bool checkTransform() const;

    std::optional<ValidTarget> runVisionPipeline(
        const std::vector<Target>& targets,
        const TransformMatrix& transform) const;

    std::vector<ValidTarget> processTargets(
        const std::vector<Target>& targets,
        const TransformMatrix& transform) const;
};