#pragma once

namespace TransformUtils {

    void rotationX(
        double degree,
        double result[3][3]);

    void rotationY(
        double degree,
        double result[3][3]);

    void rotationZ(
        double degree,
        double result[3][3]);

    void buildTransform(
        const double rotation[3][3],
        double tx,
        double ty,
        double tz,
        double transform[4][4]);

    void multiplyMatrix3x3(
        const double left[3][3],
        const double right[3][3],
        double result[3][3]);

    bool inverseTransform(
        const double transform[4][4],
        double inverse[4][4]);

    bool isValidRotationMatrix(
        const double rotation[3][3]);

} // namespace TransformUtils