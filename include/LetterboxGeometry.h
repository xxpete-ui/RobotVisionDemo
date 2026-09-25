#pragma once

#include "VisionTypes.h"

#include <algorithm>
#include <cmath>
#include <optional>

namespace LetterboxGeometry
{
    // 输入：模型的 letterbox 画布上的四条边、缩放比与左/上填边。
    // 输出：原图上的框；尺寸无效或还原后没有面积时返回 nullopt。
    inline std::optional<BoundingBox> restoreBox(
        float modelLeft,
        float modelTop,
        float modelRight,
        float modelBottom,
        float scale,
        int paddingLeft,
        int paddingTop,
        int imageWidth,
        int imageHeight)
    {
        // 先挡住非法输入，避免 NaN 进入 std::clamp，
        // 也避免 scale=0 时发生除零。
        if (!std::isfinite(modelLeft) ||
            !std::isfinite(modelTop) ||
            !std::isfinite(modelRight) ||
            !std::isfinite(modelBottom) ||
            !std::isfinite(scale) ||
            scale <= 0.0F ||
            imageWidth <= 0 ||
            imageHeight <= 0)
        {
            return std::nullopt;
        }

        // 横坐标扣左侧填边，纵坐标扣上侧填边；
        // 除以缩放比后，限制到原图范围。
        const float left = std::clamp(
            (modelLeft - static_cast<float>(paddingLeft)) / scale,
            0.0F,
            static_cast<float>(imageWidth));

        const float top = std::clamp(
            (modelTop - static_cast<float>(paddingTop)) / scale,
            0.0F,
            static_cast<float>(imageHeight));

        const float right = std::clamp(
            (modelRight - static_cast<float>(paddingLeft)) / scale,
            0.0F,
            static_cast<float>(imageWidth));

        const float bottom = std::clamp(
            (modelBottom - static_cast<float>(paddingTop)) / scale,
            0.0F,
            static_cast<float>(imageHeight));

        // 四边经裁剪后，可能都落到同一条边上。
        if (!(right > left && bottom > top))
        {
            return std::nullopt;
        }

        return BoundingBox{
            left,
            top,
            right - left,
            bottom - top
        };
    }
}