#include "FrameSync.h"

#include <cmath>

const TimedDepthFrame* findClosestDepthFrame(
    FrameTimestamp colorTimestamp,
    const std::vector<TimedDepthFrame>& depths,
    double maxDeltaMs)
{
    if (!std::isfinite(colorTimestamp.valueMs) ||
        !std::isfinite(maxDeltaMs) ||
        maxDeltaMs < 0.0)
    {
        return nullptr;
    }

    const TimedDepthFrame* best = nullptr;
    double bestDeltaMs = maxDeltaMs;

    for (const auto& depth : depths)
    {
        // 时间来源不同，或者这张深度帧的时间无效：
        // 只跳过当前候选，继续检查后面的帧。
        if (depth.timestamp.domain != colorTimestamp.domain ||
            !std::isfinite(depth.timestamp.valueMs))
        {
            continue;
        }

        const double deltaMs =
            std::abs(
                depth.timestamp.valueMs -
                colorTimestamp.valueMs);

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

// 校验深度帧是否可以直接在彩色图像素坐标 (u,v) 上采样读取深度值
bool canSampleAtColorPixels(
    const DepthFrame& depth,   // 深度图
    int colorWidth,
    int colorHeight,
    bool depthAlignedToColor) // 上游是否已把深度图对齐到彩色图像素坐标系
{
    if (!depthAlignedToColor ||
        colorWidth <= 0 ||
        colorHeight <= 0 ||
        depth.width <= 0 ||
        depth.height <= 0 ||
        depth.width != colorWidth ||
        depth.height != colorHeight)
    {
        return false;
    }

    // std::size_t无符号整数数据
    const std::size_t expectedSize =
        static_cast<std::size_t>(depth.width) *
        static_cast<std::size_t>(depth.height);

    return depth.depthMillimeters.size() == expectedSize;
}

const TimedTransform* findClosestTransform(
    FrameTimestamp imageTimestamp,
    const std::vector<TimedTransform>& transforms,
    double maxDeltaMs)
{
    if (!std::isfinite(imageTimestamp.valueMs) ||
        !std::isfinite(maxDeltaMs) ||
        maxDeltaMs < 0.0)
    {
        return nullptr;
    }

    const TimedTransform* best = nullptr;
    double bestDeltaMs = maxDeltaMs;

    for (const auto& transform : transforms)
    {
        // 时间来源不同，或者这张深度帧的时间无效：
        // 只跳过当前候选，继续检查后面的帧。
        if (transform.timestamp.domain != imageTimestamp.domain ||
            !std::isfinite(transform.timestamp.valueMs))
        {
            continue;
        }

        const double deltaMs =
            std::abs(
                transform.timestamp.valueMs -
                imageTimestamp.valueMs);

        if (deltaMs > maxDeltaMs)
        {
            continue;
        }

        if (best == nullptr || deltaMs < bestDeltaMs)
        {
            best = &transform;
            bestDeltaMs = deltaMs;
        }
    }

    return best;
}