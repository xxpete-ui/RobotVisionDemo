#pragma once
#include "VisionTypes.h"
#include <vector>


class RobotVision {
public:

    RobotVision(
        const CameraConfig& cameraConfig,
        const double T[4][4]);

    bool run(
        const std::vector<Target>& targets,
        ValidTarget& bestTarget);


    VisionStatus getStatus() const;

    const char* statusToString(VisionStatus status);

private:
    CameraConfig cameraConfig;
    double T[4][4];

    VisionStatus status;
    bool checkCameraConfig();
    
    bool checkTransform();

    bool runVisionPipeline(
        const std::vector<Target>& targets,
        ValidTarget& bestTarget);

    std::vector<ValidTarget> processTargets(const std::vector<Target>& targets);
};