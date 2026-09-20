#pragma once
#include <array>
#include <optional>
#include "VisionTypes.h"

namespace TransformUtils {

    RotationMatrix rotationX(double degree);

    RotationMatrix rotationY(double degree);

    RotationMatrix rotationZ(double degree);

    TransformMatrix buildTransform(
        const RotationMatrix& rotation,
        double tx,
        double ty,
        double tz);

    RotationMatrix multiplyMatrix3x3(
        const RotationMatrix& left,
        const RotationMatrix& right);

    [[nodiscard]]
    bool isValidRotationMatrix(
        const RotationMatrix& rotation);

    [[nodiscard]]
    std::optional<TransformMatrix> inverseTransform(
        const TransformMatrix& transform);

    [[nodiscard]]
    bool isValidTransformMatrix(
        const TransformMatrix& transform);
} // namespace TransformUtils
