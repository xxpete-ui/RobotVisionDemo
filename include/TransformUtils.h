#pragma once
#include <array>
#include "VisionTypes.h"

namespace TransformUtils {

    void rotationX(
        double degree,
        RotationMatrix& result);

    void rotationY(
        double degree,
        RotationMatrix& result);

    void rotationZ(
        double degree,
        RotationMatrix& result);

    void buildTransform(
        const RotationMatrix& rotation,
        double tx,
        double ty,
        double tz,
        TransformMatrix& transform);

    void multiplyMatrix3x3(
        const RotationMatrix& left,
        const RotationMatrix& right,
        RotationMatrix& result);

    bool isValidRotationMatrix(
        const RotationMatrix& rotation);

    bool inverseTransform(
        const TransformMatrix& transform,
        TransformMatrix& inverse);

    bool isValidTransformMatrix(
        const TransformMatrix& transform);
} // namespace TransformUtils
