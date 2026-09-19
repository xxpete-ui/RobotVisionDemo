#include "TransformUtils.h"
#include <cmath>

namespace TransformUtils
{
    void rotationX(double degree, double R[3][3])
    {
        double rad = degree * 3.14159265358979323846 / 180.0;
        double c = std::cos(rad);
        double s = std::sin(rad);

        R[0][0] = 1;
        R[0][1] = 0;
        R[0][2] = 0;

        R[1][0] = 0;
        R[1][1] = c;
        R[1][2] = -s;

        R[2][0] = 0;
        R[2][1] = s;
        R[2][2] = c;
    }

    void rotationY(double degree, double R[3][3])
    {
        double rad = degree * 3.14159265358979323846 / 180.0;
        double c = std::cos(rad);
        double s = std::sin(rad);

        R[0][0] = c;
        R[0][1] = 0;
        R[0][2] = s;

        R[1][0] = 0;
        R[1][1] = 1;
        R[1][2] = 0;

        R[2][0] = -s;
        R[2][1] = 0;
        R[2][2] = c;
    }

    void rotationZ(double degree, double R[3][3])
    {
        double rad = degree * 3.14159265358979323846 / 180.0;
        double c = std::cos(rad);
        double s = std::sin(rad);

        R[0][0] = c;
        R[0][1] = -s;
        R[0][2] = 0;

        R[1][0] = s;
        R[1][1] = c;
        R[1][2] = 0;

        R[2][0] = 0;
        R[2][1] = 0;
        R[2][2] = 1;
    }

    void multiplyMatrix3x3(const double A[3][3], const double B[3][3], double C[3][3])
    {
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                C[row][col] = 0;
                for (int k = 0; k < 3; k++)
                {
                    C[row][col] += A[row][k] * B[k][col];
                }
            }
        }
    }

    void buildTransform(const double R[3][3], double tx, double ty, double tz, double T[4][4])
    {
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                T[row][col] = R[row][col];
            }
        }

        T[0][3] = tx;
        T[1][3] = ty;
        T[2][3] = tz;

        T[3][0] = 0;
        T[3][1] = 0;
        T[3][2] = 0;
        T[3][3] = 1;
    }

    bool inverseTransform(const double T[4][4], double T_inverse[4][4])
    {
        double R[3][3];
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                R[row][col] = T[row][col];
            }
        }

        if (!isValidRotationMatrix(R))
        {
            return false;
        }

        // R^-1 = R^T 旋转矩阵的逆等于转置
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                T_inverse[row][col] = R[col][row];
            }
        }

        // 平移部分: -R^T * t
        for (int row = 0; row < 3; row++)
        {
            T_inverse[row][3] = 0;
            for (int col = 0; col < 3; col++)
            {
                T_inverse[row][3] -= T_inverse[row][col] * T[col][3];
            }
        }

        T_inverse[3][0] = 0;
        T_inverse[3][1] = 0;
        T_inverse[3][2] = 0;
        T_inverse[3][3] = 1;

        return true;
    }

    bool isValidRotationMatrix(const double R[3][3])
    {
        const double EPS = 1e-6;

        // 验证 R * R^T = I 单位矩阵
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                double sum = 0;
                for (int k = 0; k < 3; k++)
                {
                    sum += R[row][k] * R[col][k];
                }
                double expected = (row == col) ? 1.0 : 0.0;
                if (std::abs(sum - expected) > EPS)
                {
                    return false;
                }
            }
        }

        // 验证行列式 det(R) = 1
        double det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
            - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
            + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);

        if (std::abs(det - 1.0) > EPS)
        {
            return false;
        }

        return true;
    }
}
