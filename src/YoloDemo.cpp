#include "YoloDemo.h"

#include "Logger.h"
#include "RobotVision.h"
#include "VisionPipeline.h"
#include "VisionTypes.h"
#include "YoloDetector.h"
#include "TargetProcessing.h"
#include "CoordinateTransform.h"
#include "CalibrationLoader.h"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sstream>
#include <chrono>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

void runYoloDemo()
{
    const CameraConfig cameraConfig{
        2.0,
        800.0,
        800.0,
        640.0,
        360.0
    };

    const TransformMatrix transform{ {
        {{1.0, 0.0, 0.0, 0.7}},
        {{0.0, 1.0, 0.0, 2.1}},
        {{0.0, 0.0, 1.0, 3.0}},
        {{0.0, 0.0, 0.0, 1.0}}
    } };

    RobotVision vision(cameraConfig, transform);

    // 将 main.cpp 中完整的 YOLO 演示代码移动到这里。
    std::cout
        << "Begin Execution YOLO ONNX Smoke Test"
        << std::endl;

    try
    {
        const cv::Mat yoloFrame =
            cv::imread(
                "data/yolo_bus.jpg");

        if (yoloFrame.empty())
        {
            Logger::error(
                "无法读取 YOLO 测试图片");
        }
        else
        {
            constexpr int kTargetClassId = 5;

            // 构造时读取一次 ONNX 模型，指定输入 640×640、只保留 kTargetClassId
            YoloDetector yoloDetector(
                "models/yolo26n.onnx",
                640,
                640,
                kTargetClassId);

            yoloDetector.setVerbose(true);

            // 仅这个检测器打印候选框、NMS、目标细节。
            VisionPipeline yoloPipeline(
                yoloDetector,
                vision);

            // 与原图片同样宽、高、像素类型，全部填 0；用来验证“无目标”。
            const cv::Mat blankFrame(
                yoloFrame.size(),
                yoloFrame.type(),
                cv::Scalar(0, 0, 0));

            // 同一个检测器连续运行三次，并记录每次流水线耗时
            constexpr int kRepeatCount = 3;

            for (int frameIndex = 0;
                frameIndex < kRepeatCount;
                ++frameIndex)
            {
                std::cout
                    << "\n--- YOLO iteration "
                    << frameIndex + 1
                    << " ---\n";

                if (frameIndex == 1)
                {
                    // 第 1、3 次送巴士图；第 2 次送黑图。
                    yoloDetector.setFrame(blankFrame);
                    std::cout << "Input: blank frame\n";
                }
                else
                {
                    yoloDetector.setFrame(yoloFrame);
                    std::cout << "Input: bus image\n";
                }

                const auto start =
                    std::chrono::steady_clock::now();
                // 只在这里运行， 打印候选框、NMS、目标细节
                const std::optional<ValidTarget> yoloBestTarget =
                    yoloPipeline.run();

                // 获取 vision.getStatus()、打印结果、用 end-start 算本次流水线耗时。
                std::cout << std::boolalpha
                    << "Has target: "
                    << yoloBestTarget.has_value()
                    << ", status OK: "
                    << (vision.getStatus() == VisionStatus::OK)
                    << ", status NoValidTarget: "
                    << (vision.getStatus() == VisionStatus::NoValidTarget)
                    << '\n';

                const auto end =
                    std::chrono::steady_clock::now();

                // 计算 start到 end 之间消耗的时间，单位是毫秒
                const double elapsedMs =
                    std::chrono::duration<double, std::milli>(
                        end - start).count();

                std::cout
                    << "Pipeline elapsed: "
                    << elapsedMs
                    << " ms\n";

                if (yoloBestTarget)
                {
                    std::cout
                        << "YOLO best target ID: "
                        << yoloBestTarget->target.id
                        << std::endl;

                    std::cout
                        << "YOLO confidence: "
                        << yoloBestTarget->target.confidence
                        << std::endl;

                    std::cout
                        << "YOLO pixel center: ("
                        << yoloBestTarget->target.x
                        << ", "
                        << yoloBestTarget->target.y
                        << ')'
                        << std::endl;

                    std::cout
                        << "YOLO camera point: ("
                        << yoloBestTarget->cameraPoint.X
                        << ", "
                        << yoloBestTarget->cameraPoint.Y
                        << ", "
                        << yoloBestTarget->cameraPoint.Z
                        << ')'
                        << std::endl;

                    std::cout
                        << "YOLO robot point: ("
                        << yoloBestTarget->robotPoint.X
                        << ", "
                        << yoloBestTarget->robotPoint.Y
                        << ", "
                        << yoloBestTarget->robotPoint.Z
                        << ')'
                        << std::endl;
                }
                else
                {
                    Logger::warn(
                        "YOLO pipeline returned no valid target");
                }
            }

            std::cout
                << "\n--- Empty frame recovery test ---\n";

            // 1. 输入空图，应被 setFrame 拒绝。
            bool emptyFrameRejected = false;

            try
            {
                yoloDetector.setFrame(cv::Mat{});
            }
            catch (const std::invalid_argument& error)
            {
                emptyFrameRejected = true;

                std::cout
                    << "Expected setFrame rejection: "
                    << error.what()
                    << '\n';
            }

            if (!emptyFrameRejected)
            {
                throw std::runtime_error(
                    "Empty frame was unexpectedly accepted");
            }

            // 2. 此时旧帧应已清除，流水线不能继续使用它。
            bool staleFrameBlocked = false;

            try
            {
                const auto unexpectedResult =
                    yoloPipeline.run();

                // 无论返回有目标还是无目标，都不符合本次预期：
                // 没有有效图片时，应当直接拒绝推理。
                (void)unexpectedResult;
            }
            catch (const std::runtime_error& error)
            {
                staleFrameBlocked = true;

                std::cout
                    << "Expected detect rejection: "
                    << error.what()
                    << '\n';
            }

            if (!staleFrameBlocked)
            {
                throw std::runtime_error(
                    "Pipeline ran after the input frame was invalidated");
            }

            // 3. 重新提供有效图片，应能够正常恢复。
            yoloDetector.setFrame(yoloFrame);

            const std::optional<ValidTarget> recoveredTarget =
                yoloPipeline.run();

            if (!recoveredTarget ||
                vision.getStatus() != VisionStatus::OK)
            {
                throw std::runtime_error(
                    "YOLO pipeline did not recover after setting a valid frame");
            }

            std::cout
                << "Empty frame recovery: PASS\n";


            const std::vector<int>& outputShape =
                yoloDetector.getLastOutputShape();

            std::cout
                << "YOLO output shape: ";

            for (std::size_t index = 0;
                index < outputShape.size();
                ++index)
            {
                if (index > 0U)
                {
                    std::cout << " x ";
                }

                std::cout
                    << outputShape[index];
            }

            std::cout << std::endl;

            std::cout
                << "YOLO ONNX forward: PASS"
                << std::endl;
        }
    }
    catch (const cv::Exception& error)
    {
        Logger::error(
            std::string(
                "OpenCV DNN 执行失败: ") +
            error.what());
    }
    catch (const std::exception& error)
    {
        Logger::error(
            std::string(
                "YOLO Smoke Test 失败: ") +
            error.what());
    }
}


void runYoloVideoDemo()
{
    cv::VideoCapture video("data/street_cars.mp4");

    if (!video.isOpened())
    {
        std::cerr << "无法打开视频\n";
        return;
    }

    constexpr int kCarClassId = 2;
    constexpr int kMaxFrames = 100;

    YoloDetector detector(
        "models/yolo26n.onnx",
        640,
        640,
        kCarClassId);

    // 暂时沿用学习用的相机参数和变换矩阵。
    const CameraConfig camera{
        2.0, 800.0, 800.0, 640.0, 360.0
    };


    const std::string windowName = "YOLO car detection";

    cv::namedWindow(
        windowName,
        cv::WINDOW_NORMAL);

    cv::resizeWindow(
        windowName,
        960,
        540);

    for (int frameIndex = 0;
        frameIndex < kMaxFrames;
        ++frameIndex)
    {
        cv::Mat frame;

        if (!video.read(frame) || frame.empty())
        {
            std::cout
                << "视频结束或无法读取第 "
                << frameIndex + 1
                << " 帧\n";
            break;
        }

        detector.setFrame(frame);

        const auto start =
            std::chrono::steady_clock::now();

        // 当前视频没有对应的相机标定参数：只检测二维目标。
        const std::vector<Target> detectedTargets =
            detector.detect();

        const Target* bestTarget =
            TargetProcessing::selectBestTarget(detectedTargets);

        const auto end =
            std::chrono::steady_clock::now();

        const double elapsedMs =
            std::chrono::duration<double, std::milli>(
                end - start).count();

        std::cout
            << "Frame " << frameIndex + 1
            << ": " << frame.cols << "x" << frame.rows
            << ", time=" << elapsedMs << " ms"
            << ", cars=" << detector.getLastDetectionCount();;

        if (bestTarget)
        {
            const Target& target = *bestTarget;

            std::cout
                << ", classId=" << target.classId
                << ", box=("
                << target.box.x << ", "
                << target.box.y << ", "
                << target.box.width << ", "
                << target.box.height << ")"
                << ", selected center=("
                << target.x << ", "
                << target.y << ")";
        }
        else
        {
            std::cout << ", no car";
        }

        // 一帧的日志到这里结束；不能再有第二处打印 ", no car"。
        std::cout << '\n';

        // 显示和按键检查放在 if/else 外：无车帧也照常更新窗口。
        const cv::Mat debugFrame = detector.getLastDebugFrame();

        if (!debugFrame.empty())
        {
            cv::imshow(windowName, debugFrame);
        }

        const int key = cv::waitKey(1);

        if (key == 27 || key == 'q' || key == 'Q')
        {
            std::cout << "Video demo stopped by user\n";
            break;
        }
    }

    cv::destroyWindow(windowName);
}

void runCalibrationFileDemo(const std::string& path)
{
    const std::optional<CameraConfig> config =
        CalibrationLoader::load(path, 2.0);

    if (!config)
    {
        return;
    }

    std::cout << "练习标定文件: "
        << config->imageWidth << 'x'
        << config->imageHeight
        << ", fx=" << config->fx
        << ", fy=" << config->fy
        << ", cx=" << config->cx
        << ", cy=" << config->cy << '\n';

    // std::boolalpha：让 cout 打印true/false而不是 1/0
    std::cout << std::boolalpha
        << "匹配 640x480: "
        << CoordinateTransform::matchesImageSize(
            *config, 640, 480)
        << ", 匹配 1920x1080: "
        << CoordinateTransform::matchesImageSize(
            *config, 1920, 1080)
        << '\n';

    // 构造3x3相机内参矩阵 cameraMatrix（针孔模型）
    const cv::Matx33d cameraMatrix{
    config->fx, 0.0,        config->cx,
    0.0,        config->fy, config->cy,
    0.0,        0.0,        1.0
    };

    // 练习用：五个畸变系数全为零。dist = [0,0,0,0,0]
    cv::Mat distortion =
        cv::Mat::zeros(1, 5, CV_64F);
    // 练习值：只设置第一个径向畸变系数 k1。
    for (int index = 0; index < 5; ++index)
    {
        // OpenCV Mat 的`.at<T>(row,col)`，**读取 / 修改矩阵指定位置元素**
        // 这里0 = 第0行,index = 依次第 0、1、2、3、4 列
        distortion.at<double>(0, index) =
            config->distortionCoefficients[index];
    }

    // 输入像素坐标：图像上像素点 带F就是16位的浮点数
    const std::vector<cv::Point2f> pixels{
        {400.0F, 280.0F}
    };

    std::vector<cv::Point2f> normalizedPoints;

    // 核心函数 undistortPoints
    // 1:如果有畸变，先去除畸变（这里 dist 全 0，所以这一步啥也不做）
    // 2:把像素坐标，投影到归一化相机平面 Z=1
    cv::undistortPoints(
        pixels,
        normalizedPoints,
        cameraMatrix,
        distortion);

    std::cout << "归一化相机坐标: ("
        << normalizedPoints[0].x << ", "
        << normalizedPoints[0].y << ")\n";

    const double depthMeters = 2.0; // 练习值，不是深度相机读数

    std::cout << "练习相机三维点: ("
        << normalizedPoints[0].x * depthMeters << ", "
        << normalizedPoints[0].y * depthMeters << ", "
        << depthMeters << ")\n";
}