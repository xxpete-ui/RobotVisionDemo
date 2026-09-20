#pragma once

#include "VisionTypes.h"

namespace CoordinateTransform {
    bool targetToCamera(
        const Target& target,
        double Z,
        double fx,
        double fy,
        double cx,
        double cy,
        CameraPoint& result);

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