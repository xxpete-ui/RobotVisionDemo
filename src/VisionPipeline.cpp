#include "VisionPipeline.h"

#include "IDetector.h"
#include "RobotVision.h"

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
