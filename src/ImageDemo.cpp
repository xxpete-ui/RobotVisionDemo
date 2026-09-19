#include "ImageDemo.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#endif


bool showImageDemo(const std::string& imagePath) {
    cv::Mat image =cv::imread(imagePath);
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    if (image.empty())
    {
        std::cout << "fail to load image" << std::endl;
        return false;
    }
    std::cout << "================" << std::endl;
    std::cout << "Image loading successful" << std::endl;
    std::cout << "Width£º"<< image.cols << std::endl;
    std::cout << "Height£º"<< image.rows << std::endl;
    std::cout << "channels£º"<< image.channels() << std::endl;
    std::cout << "type£º"<< image.type() << std::endl;

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(640, 480));
    std::cout << "================" << std::endl;
    std::cout << "Scaled width£º"<< resized.cols << std::endl;
    std::cout << "Scaled Height£º"<< resized.rows << std::endl;
    cv::imshow("Robot Vision",image);
    cv::imshow("Resized",resized);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return true;
}
