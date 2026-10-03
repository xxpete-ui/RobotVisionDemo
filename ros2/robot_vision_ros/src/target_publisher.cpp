#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    const auto node =
        std::make_shared<rclcpp::Node>("target_publisher");

    using PointStamped = geometry_msgs::msg::PointStamped;

    const auto publisher =
        node->create_publisher<PointStamped>(
            "/vision/target_camera",
            10);

    const auto timer = node->create_wall_timer(
        std::chrono::seconds(1),
        [node, publisher]()
        {
            PointStamped message;

            // 模拟在本次回调中产生一个相机观测点。
            message.header.stamp = node->now();
            message.header.frame_id = "camera_optical_frame";

            message.point.x = 0.2;
            message.point.y = 0.1;
            message.point.z = 2.0;

            publisher->publish(message);

            RCLCPP_INFO(
                node->get_logger(),
                "Published stamp=%d.%09u point=(%.3f, %.3f, %.3f)",
                static_cast<int>(message.header.stamp.sec),
                static_cast<unsigned int>(
                    message.header.stamp.nanosec),
                message.point.x,
                message.point.y,
                message.point.z);
        });

    RCLCPP_INFO(node->get_logger(), "Target publisher started");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}