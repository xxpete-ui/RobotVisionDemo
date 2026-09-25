#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include <algorithm>
#include "VisionTypes.h"

struct DepthFrame
{
    int width{};
    int height{};

    // 按行存放：第 y 行、第 x 列对应
    // depthMillimeters[y * width + x]。
    std::vector<std::uint16_t> depthMillimeters;
};

namespace DepthSampler
{
    inline std::optional<double> sampleMeters(
        const DepthFrame& frame,
        double x,
        double y)
    {
        // 图像尺寸、数据长度和目标中心都必须有效。
        if (frame.width <= 0 ||
            frame.height <= 0 ||
            !std::isfinite(x) ||
            !std::isfinite(y) ||
            x < 0.0 ||
            y < 0.0 ||
            x >= frame.width ||
            y >= frame.height)
        {
            return std::nullopt;
        }

        const std::size_t expectedSize =
            static_cast<std::size_t>(frame.width) *
            static_cast<std::size_t>(frame.height);

        if (frame.depthMillimeters.size() != expectedSize)
        {
            return std::nullopt;
        }

        // 检测中心可能是 (1.4, 0.0)，取最近的深度像素。
        const long pixelX = std::lround(x);
        const long pixelY = std::lround(y);

        // 四舍五入后也要检查一次，例如 x=width-0.1
        // 可能被舍入到 width，已经越界。
        if (pixelX < 0 || pixelY < 0 ||
            pixelX >= frame.width ||
            pixelY >= frame.height)
        {
            return std::nullopt;
        }

        const std::size_t index =
            static_cast<std::size_t>(pixelY) *
            static_cast<std::size_t>(frame.width) +
            static_cast<std::size_t>(pixelX);

        const std::uint16_t millimeters =
            frame.depthMillimeters[index];

        // 深度图通常用 0 表示该像素没有有效距离。
        if (millimeters == 0)
        {
            return std::nullopt;
        }

        return static_cast<double>(millimeters) / 1000.0;
    }
    // 深度图常见的“空洞”：目标中心像素为 0，但周围像素有有效深度。
    inline std::optional<double> sampleWithNeighborhood(
        const DepthFrame& frame,
        double x,
        double y)
    {
        // 中心有有效深度时，直接使用它。
        if (const auto centerDepth = sampleMeters(frame, x, y))
        {
            return centerDepth;
        }

        // 中心无效时，先确认图像和坐标本身合法；
        // 越界目标不能从边缘“ 借”一个深度。
        if (frame.width <= 0 ||
            frame.height <= 0 ||
            !std::isfinite(x) ||
            !std::isfinite(y) ||
            x < 0.0 || y < 0.0 ||
            x >= frame.width || y >= frame.height ||
            frame.depthMillimeters.size() !=
            static_cast<std::size_t>(frame.width) *
            static_cast<std::size_t>(frame.height))
        {
            return std::nullopt;
        }

        const long centerX = std::lround(x);
        const long centerY = std::lround(y);

        if (centerX < 0 || centerY < 0 ||
            centerX >= frame.width ||
            centerY >= frame.height)
        {
            return std::nullopt;
        }

        std::vector<std::uint16_t> neighbors;

        // 检查中心周围 3×3 范围；图像边缘之外的像素跳过。
        for (long dy = -1; dy <= 1; ++dy)
        {
            for (long dx = -1; dx <= 1; ++dx)
            {
                const long px = centerX + dx;
                const long py = centerY + dy;

                if (px < 0 || py < 0 ||
                    px >= frame.width ||
                    py >= frame.height)
                {
                    continue;
                }

                const std::size_t index =
                    static_cast<std::size_t>(py) *
                    static_cast<std::size_t>(frame.width) +
                    static_cast<std::size_t>(px);

                const std::uint16_t value =
                    frame.depthMillimeters[index];

                if (value != 0)
                {
                    neighbors.push_back(value);
                }
            }
        }

        if (neighbors.empty())
        {
            return std::nullopt;
        }

        // 把数值从小到大排好
        std::sort(neighbors.begin(), neighbors.end());

        // ? : 是条件表达式，可以按 if/else 来读
        //数量为奇数：例如 [1000, 2000, 3000]，middle=1，直接取索引 1 的 2000。
        //数量为偶数：例如当前有 8 个值，取中间两个，
        // 即索引 middle - 1 = 3 和 middle = 4，再求平均：(2000 + 2000) / 2 = 2000 mm。
        const std::size_t middle = neighbors.size() / 2;
        const double medianMillimeters =
            neighbors.size() % 2 == 1
            ? static_cast<double>(neighbors[middle])
            : (static_cast<double>(neighbors[middle - 1]) +
                static_cast<double>(neighbors[middle])) / 2.0;

        return medianMillimeters / 1000.0;
    }

    inline std::vector<Target> attachValidDepth(
        const std::vector<Target>& detectedTargets,
        const DepthFrame& depthFrame)
    {
        std::vector<Target> targetsWithDepth;
        targetsWithDepth.reserve(detectedTargets.size());

        // detected 是循环变量：依次取出 detectedTargets 里每一个Target
        for (const Target& detected : detectedTargets)
        {
            const std::optional<double> depth =
                sampleWithNeighborhood(
                    depthFrame,
                    detected.x,
                    detected.y);

            if (!depth)
            {
                // 没有测到有效距离：不让此目标进入坐标计算。
                continue;
            }

            // 复制检测结果，保留 id、类别、框、中心和置信度；
            // 只为副本填入本次采到的深度。
            Target target = detected;
            target.depthMeters = *depth;
            targetsWithDepth.push_back(target);
        }

        return targetsWithDepth;
    }
}