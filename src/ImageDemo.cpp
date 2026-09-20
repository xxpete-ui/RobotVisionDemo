#include "ImageDemo.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/utils/logger.hpp"
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#endif


bool showImageDemo(const std::string& imagePath) {

    cv::utils::logging::setLogLevel(
        cv::utils::logging::LOG_LEVEL_WARNING);

    cv::Mat image = cv::imread(imagePath);
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    if (image.empty())
    {
        std::cout << "fail to load image" << std::endl;
        return false;
    }
    std::cout << "Width: " << image.cols << std::endl;
    std::cout << "Height: " << image.rows << std::endl;
    std::cout << "Channels: " << image.channels() << std::endl;
    std::cout << "Type: " << image.type() << std::endl;

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(640, 480));
    std::cout << "================" << std::endl;
    std::cout << "Scaled width: "
        << resized.cols
        << std::endl;

    std::cout << "Scaled height: "
        << resized.rows
        << std::endl;
    cv::imshow("Robot Vision", image);
    cv::imshow("Resized", resized);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return true;
}
