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
	}

	std::cout << "图片读取成功" << std::endl;
	std::cout << "宽度：" << image.cols << std::endl;
	std::cout << "高度：" << image.rows << std::endl;
	std::cout << "通道数：" << image.channels() << std::endl;
	std::cout << "数据类型：" << image.type() << std::endl;

	cv::imshow("robot Vision", image);
	cv::waitKey(0);

	return 0;
}