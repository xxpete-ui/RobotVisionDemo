#pragma once

#include "DepthSampler.h"

#include <vector>


struct TimedDepthFrame
{
    double timestampMs{};
    DepthFrame frame;
};


const TimedDepthFrame* findClosestDepthFrame(
    double colorTimestampMs,
    const std::vector<TimedDepthFrame>& depths,
    double maxDeltaMs);