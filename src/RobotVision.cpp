#include "RobotVision.h"

#include <cmath>
#include <iostream>


// ================================
// 数学/坐标变换工具
// ================================
void RobotVision::rotationX(
    double degree,
    double R[3][3])
{
    double rad =
        degree * 3.14159265358979323846 / 180.0;

    double c = std::cos(rad);
    double s = std::sin(rad);

    R[0][0] = 1;
    R[0][1] = 0;
    R[0][2] = 0;

    R[1][0] = 0;
    R[1][1] = c;
    R[1][2] = -s;

    R[2][0] = 0;
    R[2][1] = s;
    R[2][2] = c;
}


void RobotVision::rotationY(
    double degree,
    double R[3][3])
{
    double rad =
        degree * 3.14159265358979323846 / 180.0;

    double c = std::cos(rad);
    double s = std::sin(rad);

    R[0][0] = c;
    R[0][1] = 0;
    R[0][2] = s;

    R[1][0] = 0;
    R[1][1] = 1;
    R[1][2] = 0;

    R[2][0] = -s;
    R[2][1] = 0;
    R[2][2] = c;
}


void RobotVision::rotationZ(
    double degree,
    double R[3][3])
{
    double rad =
        degree * 3.14159265358979323846 / 180.0;

    double c = std::cos(rad);
    double s = std::sin(rad);

    R[0][0] = c;
    R[0][1] = -s;
    R[0][2] = 0;

    R[1][0] = s;
    R[1][1] = c;
    R[1][2] = 0;

    R[2][0] = 0;
    R[2][1] = 0;
    R[2][2] = 1;
}

void RobotVision::multiplyMatrix3x3(
    const double A[3][3],
    const double B[3][3],
    double C[3][3])
{
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            C[row][col] = 0;

            for (int k = 0; k < 3; k++)
            {
                C[row][col] +=
                    A[row][k] *
                    B[k][col];
            }
        }
    }
}

void RobotVision::buildTransform(
    const double R[3][3],
    double tx,
    double ty,
    double tz,
    double T[4][4])
{
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            T[row][col] =
                R[row][col];
        }
    }

    T[0][3] = tx;
    T[1][3] = ty;
    T[2][3] = tz;

    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;
}

bool RobotVision::inverseTransform(
    const double T[4][4],
    double T_inverse[4][4])
{
    double R[3][3];

    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            R[row][col] =
                T[row][col];
        }
    }

    if (!isValidRotationMatrix(R))
    {
        return false;
    }

    // R^-1 = R^T
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            T_inverse[row][col] =
                R[col][row];
        }
    }

    // -R^T * t
    for (int row = 0; row < 3; row++)
    {
        T_inverse[row][3] = 0;

        for (int col = 0; col < 3; col++)
        {
            T_inverse[row][3] -=
                T_inverse[row][col] *
                T[col][3];
        }
    }

    T_inverse[3][0] = 0;
    T_inverse[3][1] = 0;
    T_inverse[3][2] = 0;
    T_inverse[3][3] = 1;

    return true;
}


bool RobotVision::isValidRotationMatrix(
    const double R[3][3])
{
    const double EPS = 1e-6;

    // R × R^T
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            double sum = 0;

            for (int k = 0; k < 3; k++)
            {
                sum +=
                    R[row][k] *
                    R[col][k];
            }

            double expected =
                (row == col) ? 1.0 : 0.0;

            if (std::abs(sum - expected) > EPS)
            {
                return false;
            }
        }
    }

    // determinant
    double det =
        R[0][0] *
        (R[1][1] * R[2][2] -
            R[1][2] * R[2][1])

        - R[0][1] *
        (R[1][0] * R[2][2] -
            R[1][2] * R[2][0])

        + R[0][2] *
        (R[1][0] * R[2][1] -
            R[1][1] * R[2][0]);

    if (std::abs(det - 1.0) > EPS)
    {
        return false;
    }

    return true;
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


// ================================
// 目标处理
// ================================

const Target* selectBestTarget(
    const std::vector<Target>& targets)
{
    if (targets.empty())
    {
        return nullptr;
    }

    const Target* best =
        &targets[0];

    for (const auto& target : targets)
    {
        if (target.confidence >
            best->confidence)
        {
            best = &target;
        }
    }

    return best;
}


void markValidTargetGrabbed(
    std::vector<ValidTarget>& targets,
    int targetId)
{
    for (auto& target : targets)
    {
        if (target.target.id == targetId)
        {
            target.target.grabbed = true;
            return;
        }
    }
}


const ValidTarget* selectBestValidTarget(
    const std::vector<ValidTarget>& targets
) {
    if (targets.empty()) {
        return nullptr;
    }

    const ValidTarget* best = &targets[0];
    for (const auto& target : targets) {
        if (target.target.confidence > best->target.confidence) {
            best = &target;
        }
    }
    return best;
}


// ================================
// 图像/坐标处理
// ================================
bool targetToCamera(
    const Target& target,
    double Z,
    double fx,
    double fy,
    double cx,
    double cy,
    CameraPoint& result)
{
    double u = target.x;
    double v = target.y;

    if (Z <= 0 ||
        fx <= 0 ||
        fy <= 0)
    {
        std::cout
            << "相机参数错误"
            << std::endl;

        return false;
    }

    double X =
        (u - cx) * Z / fx;

    double Y =
        (v - cy) * Z / fy;

    result =
    {
        X,
        Y,
        Z
    };

    return true;
}


CameraPoint robotToCamera(
    const RobotPoint& robotPoint,
    const double T_inverse[4][4])
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
                T_inverse[row][col] *
                point[col];
        }

        result[row] +=
            T_inverse[row][3];
    }

    return
    {
        result[0],
        result[1],
        result[2]
    };
}


point2D restorePoint(
    const point2D& point,
    double scale,
    double padX,
    double padY)
{
    point2D result;

    result.x =
        (point.x - padX) / scale;

    result.y =
        (point.y - padY) / scale;

    return result;
}

RobotPoint cameraToRobot(
    const CameraPoint& cameraPoint,
    const double T[4][4])
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
                T[row][col] *
                point[col];
        }

        result[row] += T[row][3];
    }

    return
    {
        result[0],
        result[1],
        result[2]
    };
}


// ================================
// RobotVision 类
// ================================
std::vector<ValidTarget> RobotVision::processTargets(
    const std::vector<Target>& targets) {
    std::vector<ValidTarget> validTargets;
    // 相机坐标{X, Y, Z}
    
    for (const auto& target : targets) {
        CameraPoint cameraPoint;
        bool cameraPoint_result = targetToCamera(target, this->Z, this->fx, this->fy, this->cx, this->cy, cameraPoint);
        if (!cameraPoint_result) {
            std::cout << "无效目标，跳过" << std::endl;
            continue;
        }
        RobotPoint robotPoint = cameraToRobot(cameraPoint, T);
        ValidTarget validTarget;
        validTarget.target = target;
        validTarget.cameraPoint = cameraPoint;
        validTarget.robotPoint = robotPoint;
        validTargets.push_back(validTarget);
    }
    return validTargets;

}

bool RobotVision::runVisionPipeline(
    const std::vector<Target>& targets,
    ValidTarget& bestTarget
) {
    std::vector<ValidTarget> processPipeline_result = RobotVision::processTargets(targets);
    if (processPipeline_result.empty()) {
        std::cout << "没有有效目标，跳过" << std::endl;
        return false;
    }
    
    const ValidTarget* bestValidTarget = selectBestValidTarget(processPipeline_result);
    if (bestValidTarget == nullptr) {
        std::cout << "没有有效目标，跳过" << std::endl;
        return false;
    }
    int bestID = bestValidTarget->target.id;

    markValidTargetGrabbed(processPipeline_result, bestID);

    bestTarget = *bestValidTarget;

    return true;
}

RobotVision::RobotVision(
    const CameraConfig& cameraConfig,
    const double T[4][4])
    : Z(cameraConfig.Z),
     fx(cameraConfig.fx),
     fy(cameraConfig.fy),
     cx(cameraConfig.cx),
     cy(cameraConfig.cy)

{  
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            this->T[row][col] = T[row][col];
        }
    }
}

bool RobotVision::run(
    const std::vector<Target>& targets,
    ValidTarget& bestTarget)
{
    return RobotVision::runVisionPipeline(targets, bestTarget);
}