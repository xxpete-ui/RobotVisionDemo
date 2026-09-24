#include "YoloDetector.h"

#include <opencv2/core/utils/logger.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>


namespace
{
    struct DetectionCandidate
    {
        int classId{};
        float confidence{};
        cv::Rect box{};
    };

    constexpr int kClassCount = 80;
    constexpr float kConfidenceThreshold = 0.25F;
    constexpr float kNmsThreshold = 0.45F;
}

YoloDetector::YoloDetector(
    const std::string& modelPath,
    int inputWidth,
    int inputHeight,
    int targetClassId)
    : inputWidth_(inputWidth),
    inputHeight_(inputHeight),
    targetClassId_(targetClassId)

{
    if (modelPath.empty())
    {
        throw std::invalid_argument(
            "YOLO model path must not be empty");
    }

    if (inputWidth_ <= 0 ||
        inputHeight_ <= 0)
    {
        throw std::invalid_argument(
            "YOLO input size must be positive");
    }

    if (targetClassId_ < -1 || targetClassId_ >= kClassCount) {
        throw std::invalid_argument(
            "YOLO target class ID is out of range");
    }

    cv::utils::logging::setLogLevel(
        cv::utils::logging::LOG_LEVEL_WARNING);

    // 打开硬盘上的 onnx 文件，读取、解析神经网络结构 + 权重，生成可推理的 Net
    net_ = cv::dnn::readNetFromONNX(
        modelPath);

    if (net_.empty())
    {
        throw std::runtime_error(
            "Failed to load YOLO ONNX model: " +
            modelPath);
    }

    // 偏好设置 优先选用 OpenCV 自带的 DNN 算子实现
    net_.setPreferableBackend(
        cv::dnn::DNN_BACKEND_OPENCV);

    // 偏好设置 在 CPU 上执行推理
    net_.setPreferableTarget(
        cv::dnn::DNN_TARGET_CPU);
}

void YoloDetector::setFrame(
    const cv::Mat& frame)
{
    if (frame.empty())
    {
        frame_.release();

        throw std::invalid_argument(
            "YOLO input frame must not be empty");
    }

    frame_ = frame.clone();
}

std::vector<Target> YoloDetector::detect()
{
    if (frame_.empty())
    {
        throw std::runtime_error(
            "YOLO detect called before setFrame");
    }

    // 计算 Letterbox 缩放比例
    // tatic_cast<float>：强制把int转成float
    const float letterboxScale =
        std::min(
            static_cast<float>(inputWidth_) /
            static_cast<float>(frame_.cols),
            static_cast<float>(inputHeight_) /
            static_cast<float>(frame_.rows));

    // std::round：四舍五入取整，像素必须是整数
    const int resizedWidth =
        static_cast<int>(
            std::round(
                static_cast<float>(frame_.cols) *
                letterboxScale));

    const int resizedHeight =
        static_cast<int>(
            std::round(
                static_cast<float>(frame_.rows) *
                letterboxScale));

    cv::Mat resizedFrame;

    cv::resize(
        frame_,
        resizedFrame,
        cv::Size(
            resizedWidth,
            resizedHeight));

    // 计算四周填充的灰边宽度
    const int horizontalPadding =
        inputWidth_ - resizedWidth;

    const int verticalPadding =
        inputHeight_ - resizedHeight;

    const int paddingLeft =
        horizontalPadding / 2;

    const int paddingRight =
        horizontalPadding - paddingLeft;

    const int paddingTop =
        verticalPadding / 2;

    const int paddingBottom =
        verticalPadding - paddingTop;

    cv::Mat letterboxedFrame;

    // 给缩放图四周加灰边（Letterbox最终效果）
    // 参数：原图、输出图、上、下、左、右边框宽度
    cv::copyMakeBorder(
        resizedFrame,
        letterboxedFrame,
        paddingTop,
        paddingBottom,
        paddingLeft,
        paddingRight,
        cv::BORDER_CONSTANT,  // BORDER_CONSTANT：用固定颜色填充
        cv::Scalar(114, 114, 114));  // cv::Scalar(114,114,114)：YOLO标准的灰色填充（RGB都是114）

    // 图片转成神经网络输入格式（Blob）
    const cv::Mat blob =
        cv::dnn::blobFromImage(
            letterboxedFrame,
            1.0 / 255.0,  //归一化
            cv::Size(
                inputWidth_,
                inputHeight_),
            cv::Scalar(),
            true, //BGR转RGB，OpenCV默认BGR，YOLO训练用RGB
            false,  // 不交换通道
            CV_32F);  // 数据类型：32位浮点数

    // 神经网络推理
    net_.setInput(blob);

    // 执行前向推理，得到模型输出结果
    const cv::Mat output =
        net_.forward();

    lastOutputShape_.clear();   // 清空上一次的输出形状记录
    // reserve：预分配内存，提升vector插入性能
    lastOutputShape_.reserve(
        static_cast<std::size_t>(output.dims));

    // 遍历输出张量的每个维度，把维度大小存起来
    for (int dimension = 0;
        dimension < output.dims;
        ++dimension)
    {
        lastOutputShape_.push_back(
            output.size[dimension]);
    }

    // YOLO标准输出形状：[1, 4+类别数, 检测框总数]
    // dims!=3：不是三维张量，不对
    // size[0]!=1：批次不是1，不对
    // size[1]!=4+kClassCount：第二个维度不是 4个坐标+类别数，不对
    if (output.dims != 3 ||
        output.size[0] != 1 ||
        output.size[1] != 4 + kClassCount)
    {
        throw std::runtime_error(
            "Unexpected YOLO output shape");
    }

    // ===================== 9. 调整输出格式，方便遍历每个检测框 =====================
    // 把三维输出压成二维：[1, 4+类别数, 框数] → [4+类别数, 框数]
    cv::Mat predictions =
        output.reshape(
            1,
            output.size[1]);

    cv::Mat transposed;
    // 转置：[4+类别数, 框数] → [框数, 4+类别数]
    // 转置后每一行代表一个检测框，方便循环遍历每一个框
    cv::transpose(
        predictions,
        transposed);

    std::vector<DetectionCandidate> candidates;

    for (int i = 0; i < transposed.rows; ++i)
    {
        const float* prediction =
            transposed.ptr<float>(i);

        const float centerX = prediction[0];
        const float centerY = prediction[1];
        const float width = prediction[2];
        const float height = prediction[3];

        const float* classScores = prediction + 4;

        const auto bestClassIterator =
            std::max_element(
                classScores,
                classScores + kClassCount);

        const float confidence = *bestClassIterator;

        if (confidence < kConfidenceThreshold)
        {
            continue;
        }

        const int classId = static_cast<int>(
            std::distance(
                classScores,
                bestClassIterator));

        if (targetClassId_ >= 0 &&
            classId != targetClassId_)
        {
            continue;
        }

        const int left = static_cast<int>(
            centerX - width * 0.5F);

        const int top = static_cast<int>(
            centerY - height * 0.5F);

        candidates.push_back({
            classId,
            confidence,
            cv::Rect(
                left,
                top,
                static_cast<int>(width),
                static_cast<int>(height))
            });
    }

    std::cout
        << "Candidates before NMS: "
        << candidates.size()
        << '\n';


    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> classIds;

    boxes.reserve(candidates.size());
    confidences.reserve(candidates.size());
    classIds.reserve(candidates.size());

    for (const DetectionCandidate& candidate : candidates)
    {
        boxes.push_back(candidate.box);
        confidences.push_back(candidate.confidence);
        classIds.push_back(candidate.classId);
    }

    std::vector<int> selectedIndices;

    cv::dnn::NMSBoxesBatched(
        boxes,
        confidences,
        classIds,
        kConfidenceThreshold,
        kNmsThreshold,
        selectedIndices);

    // 打印日志：NMS之后剩下多少个检测结果
    std::cout
        << "Detections after NMS: "
        << selectedIndices.size()
        << '\n';

    // 坐标映射：把letterbox图的坐标 转回 原始图像坐标
    std::vector<Target> targets;
    targets.reserve(selectedIndices.size());

    // 在独立副本上画图，保留原始推理图片。
    cv::Mat debugFrame = frame_.clone();

    int nextTargetId = 1;  // 给目标编号，从1开始

    for (const int selectedIndex : selectedIndices)
    {
        const DetectionCandidate& detection =
            candidates.at(selectedIndex);

        // 取出letterbox图上框的四个边界坐标
        const float modelLeft =
            static_cast<float>(detection.box.x);

        const float modelTop =
            static_cast<float>(detection.box.y);

        const float modelRight =
            static_cast<float>(
                detection.box.x +
                detection.box.width);

        const float modelBottom =
            static_cast<float>(
                detection.box.y +
                detection.box.height);

        // 坐标反算步骤：
        // 1. 减去左边/上边的灰边填充
        // 2. 除以缩放比例，变回原图尺寸
        // 3. std::clamp：把坐标限制在原图范围内，
        // 防止越界出负数或超出图片宽高
        const float originalLeft =
            std::clamp(
                (modelLeft -
                    static_cast<float>(paddingLeft)) /
                letterboxScale,
                0.0F,
                static_cast<float>(frame_.cols));

        const float originalTop =
            std::clamp(
                (modelTop -
                    static_cast<float>(paddingTop)) /
                letterboxScale,
                0.0F,
                static_cast<float>(frame_.rows));

        const float originalRight =
            std::clamp(
                (modelRight -
                    static_cast<float>(paddingLeft)) /
                letterboxScale,
                0.0F,
                static_cast<float>(frame_.cols));

        const float originalBottom =
            std::clamp(
                (modelBottom -
                    static_cast<float>(paddingTop)) /
                letterboxScale,
                0.0F,
                static_cast<float>(frame_.rows));

        // 计算目标在原图上的中心点坐标
        const double centerX =
            (originalLeft + originalRight) * 0.5;

        const double centerY =
            (originalTop + originalBottom) * 0.5;

        // 四条边和中心点都已经是原图坐标。
        const cv::Point topLeft(
            cvRound(originalLeft),
            cvRound(originalTop));

        const cv::Point bottomRight(
            cvRound(originalRight),
            cvRound(originalBottom));

        const cv::Point center(
            cvRound(centerX),
            cvRound(centerY));

        // 绿色框。
        cv::rectangle(
            debugFrame,
            topLeft,
            bottomRight,
            cv::Scalar(0, 255, 0),
            2,
            cv::LINE_AA);

        // 红色实心中心点。
        cv::circle(
            debugFrame,
            center,
            6,
            cv::Scalar(0, 0, 255),
            cv::FILLED,
            cv::LINE_AA);

        // 组装成 Target 结构体，加入结果数组
        targets.push_back({
            nextTargetId,
            static_cast<double>(detection.confidence),
            centerX,
            centerY,
            false
            });

        std::cout
            << "targetId="
            << nextTargetId
            << ", classId="
            << detection.classId
            << ", confidence="
            << detection.confidence
            << ", center=("
            << centerX
            << ", "
            << centerY
            << "), originalBox=("
            << originalLeft
            << ", "
            << originalTop
            << ", "
            << originalRight - originalLeft
            << ", "
            << originalBottom - originalTop
            << ")\n";

        ++nextTargetId; // 目标编号自增
    }

    // 所有框画完后，只保存一次。
    const std::string debugImagePath =
        "out/yolo_detection_debug.jpg";

    if (cv::imwrite(debugImagePath, debugFrame))
    {
        std::cout
            << "YOLO debug image saved: "
            << debugImagePath
            << '\n';
    }
    else
    {
        std::cerr
            << "Failed to save YOLO debug image: "
            << debugImagePath
            << '\n';
    }


    return targets;
}

const std::vector<int>&
YoloDetector::getLastOutputShape() const noexcept
{
    return lastOutputShape_;
}
