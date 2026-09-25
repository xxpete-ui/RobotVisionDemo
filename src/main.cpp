#include <iostream>
#include <optional>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include "YoloDemo.h"
#include "ImageDemo.h"
#include "Logger.h"
#include "MockDetector.h"
#include "RobotVision.h"
#include "TargetProcessing.h"
#include "TransformTest.h"
#include "VisionPipeline.h"
#include "VisionTypes.h"


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
    MockDetector detector(targets);

    VisionPipeline pipeline(detector, vision);
    // 调用链：pipeline.run() → detector.detect()
    //      → vision.run(targets) → 返回 optional<ValidTarget>。
    const std::optional<ValidTarget> bestTarget =
        pipeline.run();

    //打印最佳目标，标记抓取，更新 MockDetector，再选下一目标。
    if (bestTarget)
    {
        std::cout << "最佳目标 ID："
            << bestTarget->target.id
            << std::endl;

        std::cout << "最佳目标置信度："
            << bestTarget->target.confidence
            << std::endl;

        std::cout << "当前抓取状态："
            << bestTarget->target.grabbed
            << std::endl;

        std::cout << "机器人抓取坐标："
            << bestTarget->robotPoint.X << ", "
            << bestTarget->robotPoint.Y << ", "
            << bestTarget->robotPoint.Z
            << std::endl;

        const bool marked =
            TargetProcessing::markTargetGrabbed(
                targets,
                bestTarget->target.id);

        std::cout << "标记是否成功："<< marked << std::endl;

        if (marked)
        {
            detector.setTargets(targets);

            const std::optional<ValidTarget> nextTarget =
                pipeline.run();

            if (nextTarget)
            {
                std::cout
                    << "下一目标 ID："
                    << nextTarget->target.id
                    << std::endl;

                std::cout
                    << "下一目标置信度："
                    << nextTarget->target.confidence
                    << std::endl;

                std::cout
                    << "下一目标机器人坐标："
                    << nextTarget->robotPoint.X << ", "
                    << nextTarget->robotPoint.Y << ", "
                    << nextTarget->robotPoint.Z
                    << std::endl;
            }
            else
            {
                Logger::warn(
                    "标记抓取后没有剩余有效目标");
            }
        }
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

    std::cout
    << "================"
    << std::endl;

    runYoloDemo();   // 第二段：静态图片、黑图、空帧恢复。
    runYoloVideoDemo();   // 第三段：街景视频最多 100 帧。


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
