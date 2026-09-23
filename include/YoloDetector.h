#pragma once

#include "IDetector.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/dnn/dnn.hpp>

#include <string>
#include <vector>

class YoloDetector final : public IDetector
{
public:
    explicit YoloDetector(
        const std::string& modelPath,
        int inputWidth = 640,
        int inputHeight = 640,
        int targetClassId = -1); // -1保留所有类别，0只保留指定类别

    void setFrame(
        const cv::Mat& frame);

    std::vector<Target> detect() override;

    const std::vector<int>&
        getLastOutputShape() const noexcept;

private:
    cv::dnn::Net net_;
    cv::Mat frame_;

    int inputWidth_;
    int inputHeight_;
    int targetClassId_;

    std::vector<int> lastOutputShape_;
};