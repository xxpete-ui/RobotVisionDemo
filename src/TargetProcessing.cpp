#include "TargetProcessing.h"


namespace TargetProcessing
{
    const Target* selectBestTarget(
        const std::vector<Target>& targets)
    {
        if (targets.empty())
        {
            return nullptr;
        }

        const Target* best =
            &targets[0];

        for (const auto& target : targets)
        {
            if (target.confidence >
                best->confidence)
            {
                best = &target;
            }
        }

        return best;
    }



    const ValidTarget* selectBestValidTarget(
        const std::vector<ValidTarget>& targets
    ) {
        if (targets.empty()) {
            return nullptr;
        }

        const ValidTarget* best = &targets[0];
        for (const auto& target : targets) {
            if (target.target.confidence > best->target.confidence) {
                best = &target;
            }
        }
        return best;
    }

    bool markTargetGrabbed(std::vector<Target>& targets, int targetId) {
        for (auto& target : targets) {
            if (target.id == targetId) {
                target.grabbed = true;
                return true;
            }
        }
        return false;
    }
}