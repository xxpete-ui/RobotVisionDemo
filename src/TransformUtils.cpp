#include "TransformUtils.h"
#include <cstddef>
#include <cmath>

namespace TransformUtils {

    RotationMatrix rotationX(double degree)
    {
        RotationMatrix result{};
        const double rad =
            degree * 3.14159265358979323846 / 180.0;

        const double c = std::cos(rad);
        const double s = std::sin(rad);

        result[0][0] = 1.0;
        result[0][1] = 0.0;
        result[0][2] = 0.0;

        result[1][0] = 0.0;
        result[1][1] = c;
        result[1][2] = -s;

        result[2][0] = 0.0;
        result[2][1] = s;
        result[2][2] = c;

        return result;
    }


    RotationMatrix rotationY(double degree)
    {
        RotationMatrix result{};
        const double rad =
            degree * 3.14159265358979323846 / 180.0;

        const double c = std::cos(rad);
        const double s = std::sin(rad);

        result[0][0] = c;
        result[0][1] = 0.0;
        result[0][2] = s;

        result[1][0] = 0.0;
        result[1][1] = 1.0;
        result[1][2] = 0.0;

        result[2][0] = -s;
        result[2][1] = 0.0;
        result[2][2] = c;

        return result;
    }


    RotationMatrix rotationZ(double degree)
    {
        RotationMatrix result{};
        const double rad =
            degree * 3.14159265358979323846 / 180.0;

        const double c = std::cos(rad);
        const double s = std::sin(rad);

        result[0][0] = c;
        result[0][1] = -s;
        result[0][2] = 0.0;

        result[1][0] = s;
        result[1][1] = c;
        result[1][2] = 0.0;

        result[2][0] = 0.0;
        result[2][1] = 0.0;
        result[2][2] = 1.0;

        return result;
    }


    RotationMatrix multiplyMatrix3x3(
        const RotationMatrix& left,
        const RotationMatrix& right)
    {
        RotationMatrix result{};

        for (std::size_t row = 0; row < result.size(); ++row)
        {
            for (std::size_t col = 0;
                col < result[row].size();
                ++col)
            {
                for (std::size_t index = 0;
                    index < left[row].size();
                    ++index)
                {
                    result[row][col] +=
                        left[row][index] *
                        right[index][col];
                }
            }
        }

        return result;
    }


    TransformMatrix buildTransform(
        const RotationMatrix& rotation, //R
        double tx,
        double ty,
        double tz)  //T
    {
        TransformMatrix transform{};

        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                transform[row][col] = rotation[row][col];
            }
        }

        transform[0][3] = tx;
        transform[1][3] = ty;
        transform[2][3] = tz;

        transform[3][0] = 0.0;
        transform[3][1] = 0.0;
        transform[3][2] = 0.0;
        transform[3][3] = 1.0;

        return transform;
    }


    std::optional<TransformMatrix> inverseTransform(
        const TransformMatrix& transform)
    {
        if (!isValidTransformMatrix(transform)) {
            return std::nullopt;
        }

        RotationMatrix R{};
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                R[row][col] = transform[row][col];
            }
        }

        if (!isValidRotationMatrix(R))
        {
            return std::nullopt;
        }

        TransformMatrix inverse{};
        // R^-1 = R^T
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                inverse[row][col] = R[col][row];
            }
        }

        // Inverse translation: -R^T * t
        for (int row = 0; row < 3; ++row)
        {
            inverse[row][3] = 0.0;

            for (int col = 0; col < 3; ++col)
            {
                inverse[row][3] -=
                    inverse[row][col] * transform[col][3];
            }
        }

        inverse[3][0] = 0.0;
        inverse[3][1] = 0.0;
        inverse[3][2] = 0.0;
        inverse[3][3] = 1.0;

        return inverse;
    }


    bool isValidRotationMatrix(
        const RotationMatrix& R)
    {
        constexpr double epsilon = 1e-6;

        // Check R * R^T = I
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                double sum = 0.0;

                for (int k = 0; k < 3; ++k)
                {
                    sum += R[row][k] * R[col][k];
                }

                const double expected =
                    (row == col) ? 1.0 : 0.0;

                if (std::abs(sum - expected) > epsilon)
                {
                    return false;
                }
            }
        }

        const double determinant =
            R[0][0] *
            (R[1][1] * R[2][2] -
                R[1][2] * R[2][1])

            - R[0][1] *
            (R[1][0] * R[2][2] -
                R[1][2] * R[2][0])

            + R[0][2] *
            (R[1][0] * R[2][1] -
                R[1][1] * R[2][0]);

        return std::abs(determinant - 1.0) <= epsilon;
    }

    bool isValidTransformMatrix(
        const TransformMatrix& transform)
    {
        constexpr double EPS = 1e-6;

        if (std::fabs(transform[3][0]) >= EPS ||
            std::fabs(transform[3][1]) >= EPS ||
            std::fabs(transform[3][2]) >= EPS ||
            std::fabs(transform[3][3] - 1.0) >= EPS)
        {
            return false;
        }

        RotationMatrix rotation{};

        for (std::size_t row = 0; row < rotation.size(); ++row)
        {
            for (std::size_t col = 0;
                col < rotation[row].size();
                ++col)
            {
                rotation[row][col] = transform[row][col];
            }
        }

        return isValidRotationMatrix(rotation);
    }
} // namespace TransformUtils
