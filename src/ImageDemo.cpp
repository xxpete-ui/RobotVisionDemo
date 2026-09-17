#include "ImageDemo.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <windows.h>


bool showImageDemo(const std::string& imagePath) {
    cv::Mat image =cv::imread(imagePath);
    SetConsoleOutputCP(CP_UTF8);
    if (image.empty())
    {
        std::cout << "fail to load image" << std::endl;
        return false;
    }
    std::cout << "================" << std::endl;
    std::cout << "Image loading successful" << std::endl;
    std::cout << "Width："<< image.cols << std::endl;
    std::cout << "Height："<< image.rows << std::endl;
    std::cout << "channels："<< image.channels() << std::endl;
    std::cout << "type："<< image.type() << std::endl;


    cv::Mat shared = image;
    cv::Mat independent = image.clone();
    std::cout << std::boolalpha;
    std::cout << "shared 是否共用图像数据："
        << (shared.data == image.data) << std::endl;
    std::cout << "clone 是否共用图像数据："
        << (independent.data == image.data) << std::endl;


    cv::Mat resized;
    cv::resize(image, resized, cv::Size(640, 480));
    std::cout << "================" << std::endl;
    std::cout << "Scaled width："<< resized.cols << std::endl;
    std::cout << "Scaled Height："<< resized.rows << std::endl;
    cv::imshow("Robot Vision",image);
    cv::imshow("Resized",resized);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return true;
}