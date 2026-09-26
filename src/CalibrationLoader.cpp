#include "CalibrationLoader.h"

#include "CoordinateTransform.h"

#include <opencv2/core.hpp>

#include <iostream>

std::optional<CameraConfig> CalibrationLoader::load(
    const std::string& path,
    double demoDepthMeters)
{
    try
    {
        const cv::FileStorage file(
            path, cv::FileStorage::READ);

        if (!file.isOpened())
        {
            std::cerr << "无法打开练习标定文件: "
                << path << '\n';
            return std::nullopt;
        }

        const char* requiredFields[] = {
            "image_width", "image_height",
            "fx", "fy", "cx", "cy", "k1",
            "k2", "p1", "p2", "k3"
        };

        for (const char* name : requiredFields)
        {
            const cv::FileNode value = file[name];

            if (value.empty())
            {
                std::cerr << "标定文件缺少字段: "
                    << name << '\n';
                return std::nullopt;
            }

            const bool isImageDimension =
                std::string(name) == "image_width" ||
                std::string(name) == "image_height";

            if (isImageDimension && !value.isInt())
            {
                std::cerr << "图像尺寸必须是整数: "
                    << name << '\n';
                return std::nullopt;
            }

            if (!isImageDimension &&
                !value.isInt() &&
                !value.isReal())
            {
                std::cerr << "标定字段不是数字: "
                    << name << '\n';
                return std::nullopt;
            }
        }

        CameraConfig config{};
        config.Z = demoDepthMeters;

        file["image_width"] >> config.imageWidth;
        file["image_height"] >> config.imageHeight;
        file["fx"] >> config.fx;
        file["fy"] >> config.fy;
        file["cx"] >> config.cx;
        file["cy"] >> config.cy;
        file["k1"] >> config.distortionCoefficients[0];
        file["k2"] >> config.distortionCoefficients[1];
        file["p1"] >> config.distortionCoefficients[2];
        file["p2"] >> config.distortionCoefficients[3];
        file["k3"] >> config.distortionCoefficients[4];


        if (!CoordinateTransform::isValidCameraConfig(config))
        {
            std::cerr << "练习标定参数无效\n";
            return std::nullopt;
        }

        return config;
    }
    catch (const cv::Exception& error)
    {
        std::cerr << "标定文件无法解析: "
            << path << '\n'
            << error.what() << '\n';
        return std::nullopt;
    }
}