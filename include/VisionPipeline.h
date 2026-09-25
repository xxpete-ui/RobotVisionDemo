#pragma once

#include "VisionTypes.h"

#include <optional>

class IDetector;
class RobotVision;
struct DepthFrame;

class VisionPipeline
{
public:
    VisionPipeline(
        IDetector& detector,
        RobotVision& robotVision);

    std::optional<ValidTarget> run();

    std::optional<ValidTarget> runWithDepth(
        const DepthFrame& depthFrame);

private:
    IDetector& detector_;
    RobotVision& robotVision_;
};