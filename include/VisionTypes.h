#pragma once

#include <array>

struct Target {
    int id;
    double confidence;
    double x;
    double y;
    bool grabbed;
};

struct CameraConfig {
    double Z;
    double fx;
    double fy;
    double cx;
    double cy;
};

struct CameraPoint {
    double X;
    double Y;
    double Z;
};

struct RobotPoint {
    double X;
    double Y;
    double Z;
};

struct ValidTarget {
    Target target;
    CameraPoint cameraPoint;
    RobotPoint robotPoint;
};

struct Point2D {
    double x;
    double y;
};

using RotationMatrix =
std::array<std::array<double, 3>, 3>;

using TransformMatrix =
std::array<std::array<double, 4>, 4>;

enum class VisionStatus {
    OK,
    InvalidCameraConfig,
    InvalidTransform,
    NoValidTarget
};