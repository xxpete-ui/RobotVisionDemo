#pragma once
#include<vector>

struct Target {
	int id;
	double confidence;
	double x;
	double y;
	bool grabbed;
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

struct point2D {
	double x;
	double y;
};

void rotationX(
	double degree,
	double R[3][3]
);

void rotationY(
	double degree,
	double R[3][3]
);

void rotationZ(
	double degree,
	double R[3][3]
);

void multiplyMatrix3x3(
	const double A[3][3],
	const double B[3][3],
	double C[3][3]
);

bool inverseTransform(
	const double T[4][4],
	double T_inverse[4][4]
);

const Target* selectBestTarget(
	const std::vector<Target>& targets
);

void markTargetGrabbed(
	std::vector<Target>& targets,
	int targetId
);

CameraPoint targetToCamera(
	const Target& target,
	double Z,
	double fx,
	double fy,
	double cx,
	double cy
);

RobotPoint cameraToRobot(
	const CameraPoint& Camerapoint,
	const double T[4][4]
);

point2D restorePoint(
	const point2D& point,
	double scale,
	double padx,
	double pady
);

void buildTransform(
	const double R[3][3],
	double tx,
	double ty,
	double tz,
	double T[4][4]
);

CameraPoint robotToCamera(
	const RobotPoint& robotPoint,
	const double T_inverse[4][4]);

bool isValidRotationMatris(
	const double R[3][3];
)