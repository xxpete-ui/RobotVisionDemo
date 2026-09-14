#include "RobotVision.h"
#include <cmath>

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