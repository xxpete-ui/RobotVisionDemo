#pragma once
#include "VisionTypes.h"
#include <vector>

namespace TargetProcessing
{
    const Target* selectBestTarget(
        const std::vector<Target>& targets);

    const ValidTarget* selectBestValidTarget(
        const std::vector<ValidTarget>& targets);

    bool markTargetGrabbed(std::vector<Target>& targets, int targetId);
}