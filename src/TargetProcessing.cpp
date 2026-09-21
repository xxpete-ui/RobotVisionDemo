#include "TargetProcessing.h"

namespace TargetProcessing
{
    const Target* selectBestTarget(
        const std::vector<Target>& targets)
    {
        const Target* bestTarget = nullptr;

        for (const auto& target : targets)
        {
            if (bestTarget == nullptr ||
                target.confidence >
                bestTarget->confidence)
            {
                bestTarget = &target;
            }
        }

        return bestTarget;
    }

    const ValidTarget* selectBestValidTarget(
        const std::vector<ValidTarget>& targets)
    {
        const ValidTarget* bestTarget = nullptr;

        for (const auto& target : targets)
        {
            if (bestTarget == nullptr ||
                target.target.confidence >
                bestTarget->target.confidence)
            {
                bestTarget = &target;
            }
        }

        return bestTarget;
    }

    bool markTargetGrabbed(
        std::vector<Target>& targets,
        int targetId)
    {
        for (auto& target : targets)
        {
            if (target.id == targetId)
            {
                target.grabbed = true;
                return true;
            }
        }

        return false;
    }
}