#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    const auto node =
        std::make_shared<rclcpp::Node>("dynamic_transform_demo");

    const auto broadcaster =
        std::make_shared<tf2_ros::TransformBroadcaster>(*node);

    const rclcpp::Time startTime = node->now();
    
    const auto timer = node->create_wall_timer(
        std::chrono::milliseconds(20),
        [node, broadcaster, startTime]()
        {
            const rclcpp::Time now = node->now();

            const double elapsedSeconds =
                (now - startTime).seconds();
            
            geometry_msgs::msg::TransformStamped transform;

            transform.header.stamp = now;
            transform.header.frame_id = "base_link";
            transform.child_frame_id = "camera_optical_frame";

            transform.transform.translation.x =
                0.7 + 0.1 * elapsedSeconds;
            
            transform.transform.translation.y = 0.2;
            transform.transform.translation.z = 0.5;

            tf2::Quaternion rotation;
            rotation.setRPY(
                0.0,
                0.0,
                1.5707963267948966);
            
            transform.transform.rotation.x = rotation.x();
            transform.transform.rotation.y = rotation.y();
            transform.transform.rotation.z = rotation.z();
            transform.transform.rotation.w = rotation.w();

            broadcaster->sendTransform(transform);
        });

    RCLCPP_INFO(node->get_logger(), "Dynamic TF demo started");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}