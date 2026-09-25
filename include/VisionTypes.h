#pragma once

#include <array>
#include <optional>

struct BoundingBox
{
    double x{};       // 原图中框的左边
    double y{};       // 原图中框的上边
    double width{};   // 原图中的宽
    double height{};  // 原图中的高
};

struct Target {
    int id;
    double confidence;
    double x;
    double y;
    bool grabbed;

    int classId{ -1 };  // -1 表示 Mock 等来源未提供类别
    BoundingBox box{};
    std::optional<double> depthMeters{};
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