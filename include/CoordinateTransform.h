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
        const double T[4][4]);


    CameraPoint robotToCamera(
        const RobotPoint& robotPoint,
        const double T_inverse[4][4]);


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