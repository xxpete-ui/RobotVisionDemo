#include "CoordinateTransform.h"
#include <cmath>
#include "Logger.h"

namespace CoordinateTransform {

    std::optional<CameraPoint> targetToCamera(
        const Target& target,
        double Z,
        double fx,
        double fy,
        double cx,
        double cy)
    {
        if (Z <= 0.0 ||
            fx <= 0.0 ||
            fy <= 0.0)
        {
            Logger::error("相机参数错误");
            return std::nullopt;
        }

        const double X =
            (target.x - cx) * Z / fx;

        const double Y =
            (target.y - cy) * Z / fy;

        return CameraPoint{
            X,
            Y,
            Z
        };
    }

    RobotPoint cameraToRobot(
        const CameraPoint& cameraPoint,
        const TransformMatrix& transform)
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
                    transform[row][col] *
                    point[col];
            }

            result[row] += transform[row][3];
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
        const TransformMatrix& inverse)
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
                    inverse[row][col] *
                    point[col];
            }

            result[row] +=
                inverse[row][3];
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