#pragma once

#include "DepthSampler.h"
#include <vector>

enum class ClockDomain
{
    VideoTimeline,
    LocalSteady
};

struct FrameTimestamp
{
    double valueMs;
    ClockDomain domain;

    FrameTimestamp(double ms, ClockDomain source)
        : valueMs(ms), domain(source)
    {
    }
};

struct TimedDepthFrame
{
    FrameTimestamp timestamp;
    DepthFrame frame;
};

struct TimedTransform
{
    FrameTimestamp timestamp;
    TransformMatrix cameraToRobot;
};

const TimedDepthFrame* findClosestDepthFrame(
    FrameTimestamp colorTimestamp,
    const std::vector<TimedDepthFrame>& depths,
    double maxDeltaMs);

bool canSampleAtColorPixels(
    const DepthFrame& depth,   // 深度图
    int colorWidth,
    int colorHeight,
    bool depthAlignedToColor); // 上游是否已把深度图对齐到彩色图像素坐标系

const TimedTransform* findClosestTransform(
    FrameTimestamp imageTimestamp,
    const std::vector<TimedTransform>& transforms,
    double maxDeltaMs);