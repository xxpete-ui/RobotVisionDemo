#include <iostream>
#include <vector>
#include <windows.h>
#include <opencv2/opencv.hpp>
#include "logger.h"
#include "RobotVision.h"
#include "TransformTest.h"


int main()
{
    SetConsoleOutputCP(CP_UTF8);
    std::cout << "测试中文 ABC 123" << std::endl;
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

    double T[4][4] =
    {
        {1, 0, 0, 0.7},
        {0, 1, 0, 2.1},
        {0, 0, 1, 3.0},
        {0, 0, 0, 1.0}
    };
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

        std::cout << "机器人抓取坐标："
            << bestTarget.robotPoint.X << ", "
            << bestTarget.robotPoint.Y << ", "
            << bestTarget.robotPoint.Z
            << std::endl;
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

    // ================================
    // 7. OpenCV 图片读取
    // ================================

    cv::Mat image =
        cv::imread(
            "data/test.jpg");


    if (image.empty())
    {
        std::cout
            << "图片读取失败"
            << std::endl;

        return 0;
    }


    std::cout
        << "================"
        << std::endl;

    std::cout
        << "图片读取成功"
        << std::endl;

    std::cout
        << "宽度："
        << image.cols
        << std::endl;

    std::cout
        << "高度："
        << image.rows
        << std::endl;

    std::cout
        << "通道数："
        << image.channels()
        << std::endl;

    std::cout
        << "图像类型："
        << image.type()
        << std::endl;


    // ================================
    // 8. Resize
    // ================================

    cv::Mat resized;

    cv::resize(
        image,
        resized,
        cv::Size(640, 480));


    std::cout
        << "================"
        << std::endl;

    std::cout
        << "缩放后宽度："
        << resized.cols
        << std::endl;

    std::cout
        << "缩放后高度："
        << resized.rows
        << std::endl;


    cv::imshow(
        "Robot Vision",
        image);

    cv::imshow(
        "Resized",
        resized);

    cv::waitKey(0);


    // ================================
    // 9. Letterbox 坐标还原
    // ================================

    double x = 450;
    double y = 200;

    double scale = 0.5;

    double padX = 0;
    double padY = 80;


    point2D result =
        restorePoint(
            { x, y },
            scale,
            padX,
            padY);


    std::cout
        << "================"
        << std::endl;

    std::cout
        << "还原后的 x："
        << result.x
        << std::endl;

    std::cout
        << "还原后的 y："
        << result.y
        << std::endl;


    // ================================
    // 10. 还原后的像素 → 相机
    // ================================

    Target detectedTarget =
    {
        100,
        0.95,
        result.x,
        result.y,
        false
    };


    CameraPoint cameraPointResult;


    bool cameraSuccess =
        targetToCamera(
            detectedTarget,
            Z,
            fx,
            fy,
            cx,
            cy,
            cameraPointResult);


    if (cameraSuccess)
    {
        std::cout
            << "================"
            << std::endl;

        std::cout
            << "Camera X："
            << cameraPointResult.X
            << std::endl;

        std::cout
            << "Camera Y："
            << cameraPointResult.Y
            << std::endl;

        std::cout
            << "Camera Z："
            << cameraPointResult.Z
            << std::endl;
    }


    // ================================
    // 11. Rz(90°) + 平移
    // ================================

    double T_1[4][4];

    double R[3][3];


    RobotVision::rotationZ(
        90.0,
        R);


    RobotVision::buildTransform(
        R,
        0.7,
        2.1,
        3.0,
        T_1);


    if (cameraSuccess)
    {
        RobotPoint robotResult = cameraToRobot(cameraPointResult, T_1);

        std::cout << "================" << std::endl;
        std::cout << "Rz + 平移后的机器人坐标：" << std::endl;
        std::cout << "x: " << robotResult.X << std::endl;
        std::cout << "y: " << robotResult.Y << std::endl;
        std::cout << "z: " << robotResult.Z << std::endl;
    }


    // ================================
    // 12. Rx + Ry + Rz
    // ================================

    double Rx[3][3];
    double Ry[3][3];
    double Rz[3][3];

    RobotVision::rotationX(
        30.0,
        Rx);

    RobotVision::rotationY(
        20.0,
        Ry);

    RobotVision::rotationZ(
        90.0,
        Rz);


    double temp[3][3];
    double R_1[3][3];


    // Rz × Ry × Rx

    RobotVision::multiplyMatrix3x3(
        Ry,
        Rx,
        temp);

    RobotVision::multiplyMatrix3x3(
        Rz,
        temp,
        R_1);


    std::cout
        << "Rz × Ry × Rx："
        << std::endl;


    for (int row = 0;
        row < 3;
        row++)
    {
        for (int col = 0;
            col < 3;
            col++)
        {
            std::cout
                << R_1[row][col]
                << "\t";
        }

        std::cout
            << std::endl;
    }


    // ================================
    // 13. Rx × Ry × Rz
    // ================================

    double temp2[3][3];
    double R_2[3][3];


    RobotVision::multiplyMatrix3x3(
        Ry,
        Rz,
        temp2);

    RobotVision::multiplyMatrix3x3(
        Rx,
        temp2,
        R_2);


    std::cout
        << "Rx × Ry × Rz："
        << std::endl;


    for (int row = 0;
        row < 3;
        row++)
    {
        for (int col = 0;
            col < 3;
            col++)
        {
            std::cout
                << R_2[row][col]
                << "\t";
        }

        std::cout
            << std::endl;
    }


    // ================================
    // 14. Transform 正逆验证
    // ================================

    double T_3[4][4] =
    {
        {0, -1, 0, 1.0},
        {1,  0, 0, 0.5},
        {0,  0, 1, 0.2},
        {0,  0, 0, 1.0}
    };


    double T_inverse[4][4];


    bool inverseSuccess = RobotVision::inverseTransform(
            T_3,
            T_inverse);


    if (inverseSuccess)
    {
        CameraPoint originalCamera =
        {
            0.2,
            0.1,
            2.0
        };


        RobotPoint robotPoint =
            cameraToRobot(
                originalCamera,
                T_3);


        std::cout
            << "================"
            << std::endl;

        std::cout
            << "Robot point: "
            << robotPoint.X
            << ", "
            << robotPoint.Y
            << ", "
            << robotPoint.Z
            << std::endl;


        CameraPoint cameraPointBack =
            robotToCamera(
                robotPoint,
                T_inverse);


        std::cout
            << "Camera point back: "
            << cameraPointBack.X
            << ", "
            << cameraPointBack.Y
            << ", "
            << cameraPointBack.Z
            << std::endl;
    }


    // ================================
    // 15. Transform 自动测试
    // ================================

    bool testResult =
        testTransform();


    std::cout
        << "================"
        << std::endl;


    if (testResult)
    {
        std::cout
            << "Transform test: PASS"
            << std::endl;
    }
    else
    {
        std::cout
            << "Transform test: FAIL"
            << std::endl;
    }


    return 0;
}