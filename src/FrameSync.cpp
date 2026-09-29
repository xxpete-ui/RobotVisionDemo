#include "FrameSync.h"
#include <cmath>

const TimedDepthFrame* findClosestDepthFrame(
	double colorTimestampMs,
	const std::vector<TimedDepthFrame>& depths,
	double maxDeltaMs)
{
    if (!std::isfinite(colorTimestampMs) ||
        !std::isfinite(maxDeltaMs) ||
        maxDeltaMs < 0.0)
    {
        return nullptr;
    }

    const TimedDepthFrame* best = nullptr;
    double bestDeltaMs = maxDeltaMs;

    for (const auto& depth : depths)
    {
        if (!std::isfinite(depth.timestampMs))
        {
            continue;
        }

        const double deltaMs =
            std::abs(depth.timestampMs - colorTimestampMs);

        if (deltaMs > maxDeltaMs)
        {
            continue;
        }

        if (best == nullptr || deltaMs < bestDeltaMs)
        {
            best = &depth;
            bestDeltaMs = deltaMs;
        }
    }

    return best;
}