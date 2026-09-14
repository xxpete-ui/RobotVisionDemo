#include <iostream>
#include <vector>
#include "RobotVision.h"
#include <opencv2/opencv.hpp>

int main() {
	std::vector<Target> targets = {
		{1, 0.72, 600, 350, false},
		{2, 0.95, 720, 400, false},
		{3, 0.82, 500, 300, false}
	};

	const Target* bestTarget = selectBestTarget(targets);
	if (bestTarget == nullptr) {
		std::cout << "本次没有发现目标，不执行" << std::endl;
		return 0;
	}
	int bestID = bestTarget->id;
	std::cout << "最佳Id为：" << bestID << std::endl;
	markTargetGrabbed(targets, bestID);
	for (const auto& target : targets) {
		std::cout << "========目前信息为==========\n"
			<< "\nId: " << target.id
			<< "\nconfidence: " << target.confidence
			<< "\nx: " << target.x
			<< "\ny: " << target.y
			<< "\ngrabbed: " << target.grabbed << std::endl;
	}

	double depth = 2.0;
	double fx = 800.0;
	double fy = 800.0;
	double cx = 640.0;
	double cy = 360.0;

	CameraPoint cameraPoint = targetToCamera(
		*bestTarget,
		depth,
		fx,
		fy,
		cx,
		cy
	);
	std::cout << "相机坐标：("
		<< cameraPoint.X << ", "
		<< cameraPoint.Y << ", "
		<< cameraPoint.Z << ")"
		<< std::endl;

	double T[4][4] = {
		{1, 0, 0, 0.7},
		{0, 1, 0, 2.1},
		{0, 0, 1, 3.0},
		{0, 0, 0, 1}
	};

	RobotPoint robotPoint = cameraToRobot(cameraPoint, T);

	std::cout << "机器人坐标:("
		<< robotPoint.X << ", "
		<< robotPoint.Y << ", "
		<< robotPoint.Z << ")"
		<< std::endl;

	cv::Mat image = cv::imread("data/test.jpg");
	if (image.empty()) {
		std::cout << "图片读取失败" << std::endl;
		return 0;
	};
	std::cout << "================" << std::endl;
	std::cout << "图片读取成功" << std::endl;
	std::cout << "宽度：" << image.cols << std::endl;
	std::cout << "高度：" << image.rows << std::endl;
	std::cout << "通道数：" << image.channels() << std::endl;
	std::cout << "数据类型：" << image.type() << std::endl;

	cv::Mat resized;

	cv::resize(
		image,
		resized,
		cv::Size(640, 480)
	);
	std::cout << "================" << std::endl;
	std::cout << "缩放后宽度：" << resized.cols << std::endl;
	std::cout << "缩放后高度：" << resized.rows << std::endl;
	cv::imshow("robot Vision", image);
	cv::imshow("Resized", resized);
	cv::waitKey(0);


	double x = 450;
	double y = 200;
	double scale = 0.5;
	double padX = 0;
	double padY = 80;
	point2D result = restorePoint({ x, y }, scale, padX, padY);
	std::cout << "================" << std::endl;
	std::cout << "x: " << result.x << std::endl;
	std::cout << "y: " << result.y << std::endl;


	Target detectedTarget = {
	100,
	0.95,
	result.x,
	result.y,
	false
	};
	CameraPoint cameraPoint_result = targetToCamera(
		detectedTarget,
		depth,
		fx,
		fy,
		cx,
		cy
	);
	std::cout << "================" << std::endl;
	std::cout << "x: " << cameraPoint_result.X << std::endl;
	std::cout << "y: " << cameraPoint_result.Y << std::endl;
	std::cout << "z: " << cameraPoint_result.Z << std::endl;

	double T_1[4][4];
	double R[3][3];
	rotationZ(90.0, R);
	buildTransform(R, 0.7, 2.1, 3.0, T_1);
	std::cout << "================" << std::endl;
	RobotPoint robotResult = cameraToRobot(cameraPoint_result, T_1);
	std::cout << "x: " << robotResult.X << std::endl;
	std::cout << "y: " << robotResult.Y << std::endl;
	std::cout << "z: " << robotResult.Z << std::endl;

	double Rx[3][3];

	rotationX(90.0, Rx);

	std::cout << "Rx = " << std::endl;

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			std::cout << Rx[row][col] << "\t";
		}

		std::cout << std::endl;
	}
	
	double Ry[3][3];

	rotationY(90.0, Ry);

	std::cout << "Ry = " << std::endl;

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			std::cout << Ry[row][col] << "\t";
		}

		std::cout << std::endl;
	}

	double A[3][3] = {
	{1, 2, 3},
	{4, 5, 6},
	{7, 8, 9}
	};

	double B[3][3] = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1}
	};

	double C[3][3];

	multiplyMatrix3x3(A, B, C);
	std::cout << "C = " << std::endl;

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			std::cout << C[row][col] << "\t";
		}

		std::cout << std::endl;
	}

	return 0;
}