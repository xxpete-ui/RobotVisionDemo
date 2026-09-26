#include "CoordinateTransform.h"
#include <cmath>
#include "Logger.h"

namespace CoordinateTransform {

    std::optional<CameraPoint> targetToCamera(
        const Target& target,
        const CameraConfig& config)
    {
        if (!isValidCameraConfig(config) ||
            !std::isfinite(target.x) ||
            !std::isfinite(target.y))
        {
            Logger::error("相机参数或目标坐标错误");
            return std::nullopt;
        }

        // 有目标深度就用目标深度；没有时暂用演示配置中的固定 Z。
        const double depth =
            target.depthMeters.value_or(config.Z);

        // 0、负数、NaN 和无穷大都不能用于像素反投影。
        if (!std::isfinite(depth) || depth <= 0.0)
        {
            Logger::error("目标深度无效");
            return std::nullopt;
        }

        const double X =
            (target.x - config.cx) *
            depth /
            config.fx;

        const double Y =
            (target.y - config.cy) *
            depth /
            config.fy;

        return CameraPoint{
            X,
            Y,
            depth
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

    bool isValidCameraConfig(
        const CameraConfig& config)
    {
        for (double coefficient : config.distortionCoefficients)
        {
            if (!std::isfinite(coefficient))
            {
                return false;
            }
        }

        return
            std::isfinite(config.Z) &&
            std::isfinite(config.fx) &&
            std::isfinite(config.fy) &&
            std::isfinite(config.cx) &&
            std::isfinite(config.cy) &&
            config.Z > 0.0 &&
            config.fx > 0.0 &&
            config.fy > 0.0 &&
            // 尺寸只能是“两个都未提供”或“两个都为正”
            (
                (config.imageWidth == 0 && config.imageHeight == 0) ||
                (config.imageWidth > 0 && config.imageHeight > 0)
                );
    }

    bool matchesImageSize(
        const CameraConfig& config,
        int imageWidth,
        int imageHeight)
    {
        return
            isValidCameraConfig(config) &&
            config.imageWidth > 0 &&
            config.imageHeight > 0 &&
            imageWidth > 0 &&
            imageHeight > 0 &&
            config.imageWidth == imageWidth &&
            config.imageHeight == imageHeight;
    }
}