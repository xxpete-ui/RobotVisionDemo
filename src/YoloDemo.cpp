#include "YoloDemo.h"

#include "Logger.h"
#include "RobotVision.h"
#include "VisionPipeline.h"
#include "VisionTypes.h"
#include "YoloDetector.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

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

            YoloDetector yoloDetector(
                "models/yolo26n.onnx",
                640,
                640,
                kTargetClassId);

            VisionPipeline yoloPipeline(
                yoloDetector,
                vision);

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

                const std::optional<ValidTarget> yoloBestTarget =
                    yoloPipeline.run();

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
