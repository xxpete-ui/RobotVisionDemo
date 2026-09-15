#pragma once

#include <vector>

// ================================
// 目标
// ================================
struct Target
{
    int id;
    double confidence;
    double x;
    double y;
    bool grabbed;
};

// ================================
// 相机坐标
// ================================
struct CameraPoint
{
    double X;
    double Y;
    double Z;
};

// ================================
// 机器人坐标
// ================================
struct RobotPoint
{
    double X;
    double Y;
    double Z;
};

struct ValidTarget {
    Target target;
    CameraPoint cameraPoint;
    RobotPoint robotPoint;
};

// ================================
// 2D 点
// ================================
struct point2D
{
    double x;
    double y;
};


// ================================
// 目标选择
// ================================
const Target* selectBestTarget(
    const std::vector<Target>& targets);


// ================================
// 标记目标已经抓取
// ================================
void markValidTargetGrabbed(
    std::vector<ValidTarget>& targets,
    int targetId);


// ================================
// 像素坐标 → 相机坐标
// ================================
bool targetToCamera(
    const Target& target,
    double Z,
    double fx,
    double fy,
    double cx,
    double cy,
    CameraPoint& result);


// ================================
// 相机坐标 → 机器人坐标
// ================================
RobotPoint cameraToRobot(
    const CameraPoint& cameraPoint,
    const double T[4][4]);


// ================================
// 机器人坐标 → 相机坐标
// ================================
CameraPoint robotToCamera(
    const RobotPoint& robotPoint,
    const double T_inverse[4][4]);


// ================================
// Letterbox 坐标还原
// ================================
point2D restorePoint(
    const point2D& point,
    double scale,
    double padX,
    double padY);


// ================================
// 旋转矩阵
// ================================
void rotationX(
    double degree,
    double R[3][3]);

void rotationY(
    double degree,
    double R[3][3]);

void rotationZ(
    double degree,
    double R[3][3]);


// ================================
// 构造 4×4 Transform
// ================================
void buildTransform(
    const double R[3][3],
    double tx,
    double ty,
    double tz,
    double T[4][4]);


// ================================
// 3×3 矩阵乘法
// ================================
void multiplyMatrix3x3(
    const double A[3][3],
    const double B[3][3],
    double C[3][3]);


// ================================
// Transform 求逆
// ================================
bool inverseTransform(
    const double T[4][4],
    double T_inverse[4][4]);


// ================================
// 旋转矩阵合法性检查
// ================================
bool isValidRotationMatrix(
    const double R[3][3]);


// ================================
// 点是否相同
// ================================
bool isSamePoint(
    const CameraPoint& a,
    const CameraPoint& b,
    double EPS);


// ================================
// Transform 测试
// ================================
bool testTransform();


const ValidTarget* selectBestValidTarget(
    const std::vector<ValidTarget>& targets
);


std::vector<ValidTarget> processTargets(
    const std::vector<Target>& targets,
    double Z,
    double fx,
    double fy,
    double cx,
    double cy,
    const double T[4][4]
);


bool runVisionPipeline(
    const std::vector<Target>& targets,
    double Z,
    double fx,
    double fy,
    double cx,
    double cy,
    const double T[4][4],
    ValidTarget& bestTarget
);