#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "CoordinateTransform.h"
#include "TransformUtils.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    const auto node =
        std::make_shared<rclcpp::Node>("target_listener");

    const double tfTimeoutSeconds = 
        node->declare_parameter<double>(
            "tf_timeout_seconds",
            0.1);

    if (!std::isfinite(tfTimeoutSeconds) ||
        tfTimeoutSeconds < 0.0)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "tf_timeout_seconds must be finite and >= 0");
        
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "TF timeout: %.3f seconds",
        tfTimeoutSeconds
    );

    const auto tfBuffer =
        std::make_shared<tf2_ros::Buffer>(node->get_clock());

    const auto tfListener =
        std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    using PointStamped = geometry_msgs::msg::PointStamped;

    const auto targetPublisher =
        node->create_publisher<PointStamped>(
            "/vision/target_base",
            10);

    const auto subscription =
        node->create_subscription<PointStamped>(
            "/vision/target_camera",
            10,
            [node, targetPublisher, tfBuffer, tfTimeoutSeconds](
                PointStamped::ConstSharedPtr msg)
            {
                if (msg->header.frame_id != "camera_optical_frame")
                {
                   RCLCPP_WARN(
                      node->get_logger(),
                      "Rejected frame: %s",
                      msg->header.frame_id.c_str());
                    return;
                }
                
                // 检查点坐标。
                if (!std::isfinite(msg->point.x) ||
                    !std::isfinite(msg->point.y) ||
                    !std::isfinite(msg->point.z))
                {
                    RCLCPP_WARN(
                        node->get_logger(),
                        "Rejected non-finite point");
                    return;
                }

                // 新增：拒绝零时间戳。
                if (msg->header.stamp.sec == 0 &&
                    msg->header.stamp.nanosec == 0)
                {
                    RCLCPP_WARN(
                        node->get_logger(),
                        "Rejected zero timestamp");
                    return;
                }

                geometry_msgs::msg::TransformStamped tfTransform;

                try
                {
                    tfTransform = tfBuffer->lookupTransform(
                        "base_link",
                        msg->header.frame_id,
                        rclcpp::Time(msg->header.stamp),
                        rclcpp::Duration::from_seconds(tfTimeoutSeconds));
                }

                catch(const tf2::TransformException& error)
                {
                    RCLCPP_WARN(
                        node->get_logger(),
                        "TF lookup failed: %s",
                        error.what());
                    return;
                }

                const auto& translation = tfTransform.transform.translation;
                const auto& rotation = tfTransform.transform.rotation;

                RCLCPP_INFO(
                    node->get_logger(),
                    "TF translation=(%.3f, %.3f, %.3f) "
                    "quaternion=(%.3f, %.3f, %.3f, %.3f)",
                    translation.x,
                    translation.y,
                    translation.z,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w
                );

                // TF 旋转字段按 x、y、z、w 顺序构造四元数。
                const tf2::Quaternion quaternion(
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w);

                // 四元数转换为 3×3 旋转矩阵。
                const tf2::Matrix3x3 tfRotation(quaternion);

                // 转为项目现有的 RotationMatrix 类型
                RotationMatrix projectRotation{};

                for (int row = 0; row < 3; ++row)
                {
                    for (int col = 0; col < 3; ++col)
                    {
                        projectRotation[row][col] = tfRotation[row][col];
                    }
                }

                // 组合旋转和平移
                const TransformMatrix cameraToBase =
                    TransformUtils::buildTransform(
                        projectRotation,
                        translation.x,
                        translation.y,
                        translation.z);
                
                if (!TransformUtils::isValidTransformMatrix(cameraToBase))
                {
                    RCLCPP_WARN(
                        node->get_logger(),
                        "Rejected invalid TF transform");
                    return;
                }

                const CameraPoint cameraPoint{
                    msg->point.x,
                    msg->point.y,
                    msg->point.z
                };

                const RobotPoint robotPoint =
                    CoordinateTransform::cameraToRobot(
                        cameraPoint,
                        cameraToBase);

                PointStamped output;

                //保留输入点对应的时刻
                output.header = msg->header;

                //坐标经过变换, 所属坐标系改为基座系
                output.header.frame_id = "base_link";

                output.point.x = robotPoint.X;
                output.point.y = robotPoint.Y;
                output.point.z = robotPoint.Z;

                targetPublisher->publish(output);


                RCLCPP_INFO(
                    node->get_logger(),
                    "robot(base_link)=(%.3f, %.3f, %.3f)",
                    robotPoint.X,
                    robotPoint.Y,
                    robotPoint.Z);

                RCLCPP_INFO(
                    node->get_logger(),
                    "frame=%s stamp=%d.%09u point=(%.3f, %.3f, %.3f)",
                    msg->header.frame_id.c_str(),
                    static_cast<int>(msg->header.stamp.sec),
                    static_cast<unsigned int>(
                        msg->header.stamp.nanosec),
                    msg->point.x,
                    msg->point.y,
                    msg->point.z);
            });

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
