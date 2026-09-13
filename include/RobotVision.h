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
