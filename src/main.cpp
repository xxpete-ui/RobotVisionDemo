#include <iostream>
#include <vector>
#include "RobotVision.h"
#include <opencv2/opencv.hpp>
#include <windows.h>

int main()
{
    // 设置 Windows 控制台使用 UTF-8，避免中文乱码
    SetConsoleOutputCP(CP_UTF8);

    // ================================
    // 1. 目标检测结果
    // ================================

    std::vector<Target> targets = {
        {1, 0.72, 600, 350, false},
        {2, 0.95, 720, 400, false},
        {3, 0.82, 500, 300, false}
    };

    const Target* bestTarget = selectBestTarget(targets);

    if (bestTarget == nullptr)
    {
        std::cout << "当前没有发现目标，不执行" << std::endl;
        return 0;
    }

    int bestID = bestTarget->id;

    std::cout << "最佳Id为：" << bestID << std::endl;

    markTargetGrabbed(targets, bestID);

    for (const auto& target : targets)
    {
        std::cout << "========目前信息为==========\n"
            << "\nId: " << target.id
            << "\nconfidence: " << target.confidence
            << "\nx: " << target.x
            << "\ny: " << target.y
            << "\ngrabbed: " << target.grabbed
            << std::endl;
    }


    // ================================
    // 2. 像素坐标 → 相机坐标
    // ================================

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


    // ================================
    // 3. 相机坐标 → 机器人坐标
    //    纯平移
    // ================================

    double T[4][4] = {
        {1, 0, 0, 0.7},
        {0, 1, 0, 2.1},
        {0, 0, 1, 3.0},
        {0, 0, 0, 1}
    };

    RobotPoint robotPoint = cameraToRobot(
        cameraPoint,
        T
    );

    std::cout << "机器人坐标：("
        << robotPoint.X << ", "
        << robotPoint.Y << ", "
        << robotPoint.Z << ")"
        << std::endl;


    // ================================
    // 4. OpenCV 图像读取
    // ================================

    cv::Mat image = cv::imread("data/test.jpg");

    if (image.empty())
    {
        std::cout << "图片读取失败" << std::endl;
        return 0;
    }

    std::cout << "================" << std::endl;

    std::cout << "图片读取成功" << std::endl;
    std::cout << "宽度：" << image.cols << std::endl;
    std::cout << "高度：" << image.rows << std::endl;
    std::cout << "通道数：" << image.channels() << std::endl;
    std::cout << "图像类型：" << image.type() << std::endl;


    // ================================
    // 5. OpenCV Resize
    // ================================

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


    // ================================
    // 6. Letterbox 坐标还原
    // ================================

    double x = 450;
    double y = 200;

    double scale = 0.5;

    double padX = 0;
    double padY = 80;

    point2D result = restorePoint(
        { x, y },
        scale,
        padX,
        padY
    );

    std::cout << "================" << std::endl;

    std::cout << "x: " << result.x << std::endl;
    std::cout << "y: " << result.y << std::endl;


    // ================================
    // 7. 还原后的像素坐标 → 相机坐标
    // ================================

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


    // ================================
    // 8. Rz(90°) + 平移
    // ================================

    double T_1[4][4];

    double R[3][3];

    rotationZ(90.0, R);

    buildTransform(
        R,
        0.7,
        2.1,
        3.0,
        T_1
    );

    std::cout << "================" << std::endl;

    RobotPoint robotResult = cameraToRobot(
        cameraPoint_result,
        T_1
    );

    std::cout << "x: " << robotResult.X << std::endl;
    std::cout << "y: " << robotResult.Y << std::endl;
    std::cout << "z: " << robotResult.Z << std::endl;


    // ================================
    // 9. Rx + Ry + Rz 矩阵组合
    //    R = Rz × Ry × Rx
    // ================================

    double Rx[3][3];
    double Ry[3][3];
    double Rz[3][3];

    rotationX(30.0, Rx);
    rotationY(20.0, Ry);
    rotationZ(90.0, Rz);

    double temp[3][3];
    double R_1[3][3];

    // R_1 = Rz × Ry × Rx
    multiplyMatrix3x3(Ry, Rx, temp);
    multiplyMatrix3x3(Rz, temp, R_1);

    std::cout << "Rz × Ry × Rx：" << std::endl;

    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            std::cout << R_1[row][col] << "\t";
        }

        std::cout << std::endl;
    }


    // ======================================
    // 对比：Rx × Ry × Rz
    // ======================================

    double temp2[3][3];
    double R_2[3][3];

    // temp2 = Ry × Rz
    multiplyMatrix3x3(Ry, Rz, temp2);

    // R_2 = Rx × temp2
    //     = Rx × Ry × Rz
    multiplyMatrix3x3(Rx, temp2, R_2);

    std::cout << "Rx × Ry × Rz：" << std::endl;

    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            std::cout << R_2[row][col] << "\t";
        }

        std::cout << std::endl;
    }
    
    double T_3[4][4] =
    {
        {0, -1, 0, 1.0},
        {1,  0, 0, 0.5},
        {0,  0, 1, 0.2},
        {0,  0, 0, 1.0}
    };
    double T_inverse[4][4];

    inverseTransform(T_3, T_inverse);
    cameraPoint = {
    0.2,
    0.1,
    2.0
    };
    robotPoint = cameraToRobot(cameraPoint, T_3);

    std::cout << "Robot point: "
        << robotPoint.X << ", "
        << robotPoint.Y << ", "
        << robotPoint.Z << std::endl;

    CameraPoint cameraPointBack =
        robotToCamera(robotPoint, T_inverse);

    std::cout << "Camera point back: "
        << cameraPointBack.X << ", "
        << cameraPointBack.Y << ", "
        << cameraPointBack.Z << std::endl;

    return 0;
}

