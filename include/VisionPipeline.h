#pragma once

#include "VisionTypes.h"

#include <optional>

class IDetector;
class RobotVision;

class VisionPipeline
{
public:
    VisionPipeline(
        IDetector& detector,
        RobotVision& robotVision);

    std::optional<ValidTarget> run();

private:
    IDetector& detector_;
    RobotVision& robotVision_;
};