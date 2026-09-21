#pragma once

#include "VisionTypes.h"

#include <vector>

namespace TargetProcessing
{
    [[nodiscard]]
    const Target* selectBestTarget(
        const std::vector<Target>& targets);

    [[nodiscard]]
    const ValidTarget* selectBestValidTarget(
        const std::vector<ValidTarget>& targets);

    [[nodiscard]]
    bool markTargetGrabbed(
        std::vector<Target>& targets,
        int targetId);
}