#include "RobotVision.h"


const Target* selectBestTarget(const std::vector<Target>& targets) {
	/*找出置信度最高的目标
	如果没有目标，返回 nullptr
	找到以后，不能复制整个 Target，返回这个目标的地址
	后面还要修改原 targets 中这个目标的 grabbed*/
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
	//找到id == targetId的目标，把它的grabbed改成true
	for (int i = 0; i < targets.size(); i++) {
		if (targets[i].id == targetId) {
			targets[i].grabbed = true;
			return;
		}
	}
}

CameraPoint targetToCamera(const Target& target, double Z,
	double fx,
	double fy,
	double cx,
	double cy) {
	double u = target.x;
	double v = target.y;

	double X = (u - cx) * Z / fx;
	double Y = (v - cy) * Z / fy;

	return{ X, Y, Z };
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
			// row -> 行
			// col -> 列
			result[row] += T[row][col] * point[col];
		}
		result[row] += T[row][3];
	}

	return { result[0], result[1], result[2] };
}