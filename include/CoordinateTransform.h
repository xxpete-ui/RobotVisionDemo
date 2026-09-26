#pragma once
#include <optional>
#include "VisionTypes.h"

namespace CoordinateTransform
{
    [[nodiscard]]
    std::optional<CameraPoint> targetToCamera(
        const Target& target,
        const CameraConfig& config);

    RobotPoint cameraToRobot(
        const CameraPoint& cameraPoint,
        const TransformMatrix& transform);

    CameraPoint robotToCamera(
        const RobotPoint& robotPoint,
        const TransformMatrix& inverse);


    Point2D restorePoint(
        const Point2D& point,
        double scale,
        double padX,
        double padY);


    bool isSamePoint(
        const CameraPoint& a,
        const CameraPoint& b,
        double EPS);

    [[nodiscard]]
    bool isValidCameraConfig(
        const CameraConfig& config);

    bool matchesImageSize(
        const CameraConfig& config,
        int imageWidth,
        int imageHeight);
}