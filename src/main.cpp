#include <iostream>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#endif
#include <opencv2/opencv.hpp>
#include "Logger.h"
#include "RobotVision.h"
#include "VisionTypes.h"
#include "TargetProcessing.h"
#include "ImageDemo.h"
#include "TransformTest.h"


int main()
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    //std::cout << "当前工作目录：" << std::filesystem::current_path() << std::endl;
    //std::cout << "图片是否存在："<< std::filesystem::exists("data/test.jpg") << std::endl;
    Logger::info("机器人视觉程序启动"); 
    
    std::vector<Target> targets =
    {
        {1, 0.72, 600, 350, false},
        {2, 0.75, 720, 400, false},
        {3, 0.68, 500, 300, false}
    };

    double Z = 2.0;
    double fx = 800.0;
    double fy = 800.0;
    double cx = 640.0;
    double cy = 360.0;

    TransformMatrix T{ {
    {{1, 0, 0, 0.7}},
    {{0, 1, 0, 2.1}},
    {{0, 0, 1, 3.0}},
    {{0, 0, 0, 1.0}}
} };
    CameraConfig cameraConfig = { Z, fx, fy, cx, cy };
   
    RobotVision vision(cameraConfig, T);
    ValidTarget bestTarget{};
    
    if (vision.run(targets, bestTarget))
    {
        std::cout << "最佳目标 ID："
            << bestTarget.target.id
            << std::endl;

        std::cout << "最佳目标置信度："
            << bestTarget.target.confidence
            << std::endl;

        std::cout << "当前抓取状态："
            << bestTarget.target.grabbed
            << std::endl;

        std::cout << "机器人抓取坐标："
            << bestTarget.robotPoint.X << ", "
            << bestTarget.robotPoint.Y << ", "
            << bestTarget.robotPoint.Z
            << std::endl;

        bool marked = TargetProcessing::markTargetGrabbed(targets, bestTarget.target.id);
        std::cout << "标记是否成功：" << marked << std::endl;
    }
    else {
        switch (vision.getStatus()) {
             case VisionStatus::InvalidCameraConfig:
                  Logger::error("相机参数错误");
                  std::cout << "相机参数错误" << std::endl;
                  break;
             case VisionStatus::InvalidTransform:
                  std::cout << "变换矩阵错误" << std::endl;
                  break;
             case VisionStatus::NoValidTarget:
                  std::cout << "当前没有有效目标，继续下一帧" << std::endl;
                  break;
        }
        
    }

    if (!showImageDemo("data/test.jpg")) {
        return 0;
    }


    CameraPoint cameraPointResult{};
    bool cameraSuccess = demoLetterboxToCamera(cameraConfig, cameraPointResult);

    if (cameraSuccess) {
        printRotateRobotPoint(cameraPointResult);
    }
    printRotationComposition();

    bool testResult = testTransform();
    std::cout << "================" << std::endl;
    if (testResult){
        std::cout << "Transform test: PASS" << std::endl;}
    else{
        std::cout << "Transform test: FAIL" << std::endl;}
    return 0;
}