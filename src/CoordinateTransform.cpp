#include "CoordinateTransform.h"
#include <cmath>
#include "Logger.h"

namespace CoordinateTransform {

    bool targetToCamera(
        const Target& target,
        double Z,
        double fx,
        double fy,
        double cx,
        double cy,
        CameraPoint& result)
    {
        double u = target.x;
        double v = target.y;

        if (Z <= 0 ||
            fx <= 0 ||
            fy <= 0)
        {
            Logger::error("相机参数错误");
            return false;
        }

        double X =
            (u - cx) * Z / fx;

        double Y =
            (v - cy) * Z / fy;

        result =
        {
            X,
            Y,
            Z
        };

        return true;
    }

    RobotPoint cameraToRobot(
        const CameraPoint& cameraPoint,
        const double T[4][4])
    {
        double result[3] =
        {
            0.0,
            0.0,
            0.0
        };

        double point[3] =
        {
            cameraPoint.X,
            cameraPoint.Y,
            cameraPoint.Z
        };

        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                result[row] +=
                    T[row][col] *
                    point[col];
            }

            result[row] += T[row][3];
        }

        return
        {
            result[0],
            result[1],
            result[2]
        };
    }

    CameraPoint robotToCamera(
        const RobotPoint& robotPoint,
        const double T_inverse[4][4])
    {
        double result[3] =
        {
            0.0,
            0.0,
            0.0
        };

        double point[3] =
        {
            robotPoint.X,
            robotPoint.Y,
            robotPoint.Z
        };

        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                result[row] +=
                    T_inverse[row][col] *
                    point[col];
            }

            result[row] +=
                T_inverse[row][3];
        }

        return
        {
            result[0],
            result[1],
            result[2]
        };
    }

    Point2D restorePoint(
        const Point2D& point,
        double scale,
        double padX,
        double padY)
    {
        Point2D result;

        result.x =
            (point.x - padX) / scale;

        result.y =
            (point.y - padY) / scale;

        return result;
    }

    bool isSamePoint(
        const CameraPoint& a,
        const CameraPoint& b,
        double EPS)
    {
        return
            std::abs(a.X - b.X) < EPS &&
            std::abs(a.Y - b.Y) < EPS &&
            std::abs(a.Z - b.Z) < EPS;
    }
}