#include "RobotVision.h"
#include <cmath>
#include <cassert>
#include <iostream>

const Target* selectBestTarget(const std::vector<Target>& targets) {
	if (targets.empty()) {
		return nullptr;
	};
	const Target* bestConfidence = &targets[0];
	for (const auto& t : targets) {
		if (t.confidence > bestConfidence->confidence) {
			bestConfidence = &t;
		}
	}
	return bestConfidence;

}

void markTargetGrabbed(std::vector<Target>& targets, int targetId) {
	for (int i = 0; i < targets.size(); i++) {
		if (targets[i].id == targetId) {
			targets[i].grabbed = true;
			return;
		}
	}
}

bool targetToCamera(const Target& target, double Z,
	double fx,
	double fy,
	double cx,
	double cy,
	CameraPoint& result) 

{
	double u = target.x;
	double v = target.y;

	if (Z <= 0 || fx <= 0 || fy <= 0) {
		std::cout << "error something was happend" << std::endl;
		return false;
	}

	double X = (u - cx) * Z / fx;
	double Y = (v - cy) * Z / fy;

	result = { X, Y, Z };
	return true;
}

RobotPoint cameraToRobot(const CameraPoint& camerapoint, const double T[4][4]) {
	//TODO
	double result[3] =
	{
		0.0,
		0.0,
		0.0
	};
	double point[3] =
	{
		camerapoint.X,
		camerapoint.Y,
		camerapoint.Z
	};
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			result[row] += T[row][col] * point[col];
		}
		result[row] += T[row][3];
	}

	return { result[0], result[1], result[2] };
}

point2D restorePoint(
	const point2D& point,
	double scale,
	double padX,
	double padY)
{
	double x = (point.x - padX) / scale;
	double y = (point.y - padY) / scale;

	return{ x, y };
}

void rotationX(
	double degree,
	double R[3][3])
{
	double rad = degree * 3.141592653589793 / 180.0;

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

void rotationY(
	double degree,
	double R[3][3])
{
	double rad = degree * 3.141592653589793 / 180.0;

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

void rotationZ(
	double degree,
	double R[3][3]
) {
	double rad = degree * 3.141592653589793 / 180.0;

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

void buildTransform(
	const double R[3][3],
	double tx,
	double ty,
	double tz,
	double T[4][4]
)
{
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++)
		{
			T[row][col] = R[row][col];
		}
	}
	T[0][3] = tx;
	T[1][3] = ty;
	T[2][3] = tz;

	T[3][0] = 0;
	T[3][1] = 0;
	T[3][2] = 0;
	T[3][1] = 1;
}

void multiplyMatrix3x3(
	const double A[3][3],
	const double B[3][3],
	double C[3][3]) {
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			C[row][col] = 0.0;
			for (int k = 0; k < 3; k++) {
				C[row][col] += A[row][k] * B[k][col];
			}
		}
	}
}

bool inverseTransform(
	const double T[4][4],
	double T_inverse[4][4]
) {
	//TODO 提取旋转矩阵R
	
	double R[3][3] = {
		{T[0][0], T[0][1], T[0][2]},
		{T[1][0], T[1][1], T[1][2]},
		{T[2][0], T[2][1], T[2][2]}
	};

	if (!isValidRotationMatrix(R)) {
		return false;
	}

	double t[3] = { T[0][3], T[1][3], T[2][3] };

	 // 计算R的转置

	double RT[3][3] = {
		{T[0][0], T[1][0], T[2][0]},
		{T[0][1], T[1][1], T[2][1]},
		{T[0][2], T[1][2], T[2][2]}
	};

	//TODO 计算逆变换的平移
	double neg_Rt[3];
	neg_Rt[0] = -(RT[0][0] * t[0] + RT[0][1] * t[1] + RT[0][2] * t[2]);
	neg_Rt[1] = -(RT[1][0] * t[0] + RT[1][1] * t[1] + RT[1][2] * t[2]);
	neg_Rt[2] = -(RT[2][0] * t[0] + RT[2][1] * t[1] + RT[2][2] * t[2]);

	//TODO 最后一行
	T_inverse[0][0] = RT[0][0];
	T_inverse[0][1] = RT[0][1];
	T_inverse[0][2] = RT[0][2];
	T_inverse[0][3] = neg_Rt[0];
	T_inverse[1][0] = RT[1][0];
	T_inverse[1][1] = RT[1][1];
	T_inverse[1][2] = RT[1][2];
	T_inverse[1][3] = neg_Rt[1];
	T_inverse[2][0] = RT[2][0];
	T_inverse[2][1] = RT[2][1];
	T_inverse[2][2] = RT[2][2];
	T_inverse[2][3] = neg_Rt[2];
	T_inverse[3][0] = 0.0;
	T_inverse[3][1] = 0.0;
	T_inverse[3][2] = 0.0;
	T_inverse[3][3] = 1.0;

	return true;
}

CameraPoint robotToCamera(
	const RobotPoint& robotPoint,
	const double T_inverse[4][4])
{
	double result[3] = { 0.0, 0.0, 0.0 };

	double point[3] = {
		robotPoint.X,
		robotPoint.Y,
		robotPoint.Z
	};

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			result[row] +=
				T_inverse[row][col] * point[col];
		}

		result[row] += T_inverse[row][3];
	}

	return {
		result[0],
		result[1],
		result[2]
	};
}

bool isValidRotationMatrix(
	const double R[3][3]
) {
	const double EPS = 1e-6;
	double RT[3][3];
	RT[0][0] = R[0][0]; RT[0][1] = R[1][0]; RT[0][2] = R[2][0];
	RT[1][0] = R[0][1]; RT[1][1] = R[1][1]; RT[1][2] = R[2][1];
	RT[2][0] = R[0][2]; RT[2][1] = R[1][2]; RT[2][2] = R[2][2];

	double R_result[3][3];
	R_result[0][0] = (RT[0][0] * R[0][0] + RT[0][1] * R[1][0] + RT[0][2] * R[2][0]);
	R_result[0][1] = (RT[0][0] * R[0][1] + RT[0][1] * R[1][1] + RT[0][2] * R[2][1]);
	R_result[0][2] = (RT[0][0] * R[0][2] + RT[0][1] * R[1][2] + RT[0][2] * R[2][2]);

	R_result[1][0] = (RT[1][0] * R[0][0] + RT[1][1] * R[1][0] + RT[1][2] * R[2][0]);
	R_result[1][1] = (RT[1][0] * R[0][1] + RT[1][1] * R[1][1] + RT[1][2] * R[2][1]);
	R_result[1][2] = (RT[1][0] * R[0][2] + RT[1][1] * R[1][2] + RT[1][2] * R[2][2]);

	R_result[2][0] = (RT[2][0] * R[0][0] + RT[2][1] * R[1][0] + RT[2][2] * R[2][0]);
	R_result[2][1] = (RT[2][0] * R[0][1] + RT[2][1] * R[1][1] + RT[2][2] * R[2][1]);
	R_result[2][2] = (RT[2][0] * R[0][2] + RT[2][1] * R[1][2] + RT[2][2] * R[2][2]);

	//判断R_result是不是接近单位矩阵
	bool ok = true;
	if (fabs(R_result[0][0] - 1.0) > EPS) {
		ok = false;
	}
	if (fabs(R_result[1][1] - 1.0) > EPS) {
		ok = false;
	}
	if (fabs(R_result[2][2] - 1.0) > EPS) {
		ok = false;
	}
	if (fabs(R_result[0][1]) > EPS) {
		ok = false;
	}
	if (fabs(R_result[0][2]) > EPS) {
		ok = false;
	}
	if (fabs(R_result[1][0]) > EPS) {
		ok = false;
	}
	if (fabs(R_result[1][2]) > EPS) {
		ok = false;
	}
	if (fabs(R_result[2][0]) > EPS) {
		ok = false;
	}
	if (fabs(R_result[2][1]) > EPS) {
		ok = false;
	}

	double det =
		R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
		- R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
		+ R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);

	if (fabs(det - 1.0) > EPS)
	{
		ok = false;
	}

	return ok;
}

bool isSamePoint(
	const CameraPoint& a,
	const CameraPoint& b,
	double EPS
) {
	//TODO
	bool success = true;
	if (fabs(a.X) - fabs(b.X) > EPS) {
		success = false;
	}
	if (fabs(a.Y) - fabs(b.Y) > EPS) {
		success = false;
	}
	if (fabs(a.Y) - fabs(b.Y) > EPS) {
		success = false;
	}
	return success;
}