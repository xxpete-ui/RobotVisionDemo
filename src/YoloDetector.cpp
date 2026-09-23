#include "YoloDetector.h"
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/imgproc.hpp>

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

    net_ = cv::dnn::readNetFromONNX(
        modelPath);

    if (net_.empty())
    {
        throw std::runtime_error(
            "Failed to load YOLO ONNX model: " +
            modelPath);
    }

    net_.setPreferableBackend(
        cv::dnn::DNN_BACKEND_OPENCV);

    net_.setPreferableTarget(
        cv::dnn::DNN_TARGET_CPU);
}

void YoloDetector::setFrame(
    const cv::Mat& frame)
{
    if (frame.empty())
    {
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

    const float letterboxScale =
        std::min(
            static_cast<float>(inputWidth_) /
            static_cast<float>(frame_.cols),
            static_cast<float>(inputHeight_) /
            static_cast<float>(frame_.rows));

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

    cv::copyMakeBorder(
        resizedFrame,
        letterboxedFrame,
        paddingTop,
        paddingBottom,
        paddingLeft,
        paddingRight,
        cv::BORDER_CONSTANT,
        cv::Scalar(114, 114, 114));

    const cv::Mat blob =
        cv::dnn::blobFromImage(
            letterboxedFrame,
            1.0 / 255.0,
            cv::Size(
                inputWidth_,
                inputHeight_),
            cv::Scalar(),
            true,
            false,
            CV_32F);

    net_.setInput(blob);

    const cv::Mat output =
        net_.forward();

    lastOutputShape_.clear();
    lastOutputShape_.reserve(
        static_cast<std::size_t>(output.dims));

    for (int dimension = 0;
        dimension < output.dims;
        ++dimension)
    {
        lastOutputShape_.push_back(
            output.size[dimension]);
    }

    if (output.dims != 3 ||
        output.size[0] != 1 ||
        output.size[1] != 4 + kClassCount)
    {
        throw std::runtime_error(
            "Unexpected YOLO output shape");
    }

    cv::Mat predictions =
        output.reshape(
            1,
            output.size[1]);

    cv::Mat transposed;
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

    std::cout
        << "Detections after NMS: "
        << selectedIndices.size()
        << '\n';

    std::vector<Target> targets;
    targets.reserve(selectedIndices.size());

    int nextTargetId = 1;

    for (const int selectedIndex : selectedIndices)
    {
        const DetectionCandidate& detection =
            candidates.at(selectedIndex);

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

        const double centerX =
            (originalLeft + originalRight) * 0.5;

        const double centerY =
            (originalTop + originalBottom) * 0.5;

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

        ++nextTargetId;
    }

    return targets;
}

const std::vector<int>&
YoloDetector::getLastOutputShape() const noexcept
{
    return lastOutputShape_;
}