#pragma once
#include <vector>

struct Target
{
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

struct CameraPoint
{
    double X;
    double Y;
    double Z;
};

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

struct point2D
{
    double x;
    double y;
};

enum class VisionStatus
{
    OK,
    InvalidCameraConfig,
    InvalidTransform,
    NoValidTarget
};


const Target* selectBestTarget(
    const std::vector<Target>& targets);

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


point2D restorePoint(
    const point2D& point,
    double scale,
    double padX,
    double padY);


bool isSamePoint(
    const CameraPoint& a,
    const CameraPoint& b,
    double EPS);


const ValidTarget* selectBestValidTarget(
    const std::vector<ValidTarget>& targets);


bool markTargetGrabbed(std::vector<Target>& targets, int targetId);


class RobotVision {
public:

    RobotVision(
        const CameraConfig& cameraConfig,
        const double T[4][4]);

    bool run(
        const std::vector<Target>& targets,
        ValidTarget& bestTarget);

    static void rotationX(
        double degree,
        double R[3][3]);

    static void rotationY(
        double degree,
        double R[3][3]);

    static void rotationZ(
        double degree,
        double R[3][3]);

    static void buildTransform(
        const double R[3][3],
        double tx,
        double ty,
        double tz,
        double T[4][4]);

    static void multiplyMatrix3x3(
        const double A[3][3],
        const double B[3][3],
        double C[3][3]);

    static bool inverseTransform(
        const double T[4][4],
        double T_inverse[4][4]);

    static bool isValidRotationMatrix(
        const double R[3][3]);

    VisionStatus getStatus() const;

    const char* statusToString(VisionStatus status);

private:
    double Z;
    double fx;
    double fy;
    double cx;
    double cy;
    double T[4][4];

    VisionStatus status;
    bool checkCameraConfig();
    
    bool checkTransform();

    bool runVisionPipeline(
        const std::vector<Target>& targets,
        ValidTarget& bestTarget);

    std::vector<ValidTarget> processTargets(const std::vector<Target>& targets);
};