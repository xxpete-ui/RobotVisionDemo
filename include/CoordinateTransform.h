#pragma once
#include <optional>
#include "VisionTypes.h"

namespace CoordinateTransform {
    std::optional<CameraPoint> targetToCamera(
        const Target& target,
        double Z,
        double fx,
        double fy,
        double cx,
        double cy);

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
}