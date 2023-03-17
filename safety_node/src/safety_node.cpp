#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"

class SafetyNode : public rclcpp::Node {
public:
    SafetyNode() : Node("safety_node") {
        drive_command_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&SafetyNode::scan_callback, this, std::placeholders::_1));
        odom_car_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&SafetyNode::odom_callback, this, std::placeholders::_1));
        speed_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Emergency braking node started!");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        int data_size = static_cast<int>((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        for (int i = 0; i < data_size; i++) {
            float range = scan_msg->ranges[i];
            float range_speed = speed_ * std::cos(i * scan_msg->angle_increment + scan_msg->angle_min);
            if (range_speed == 0) {
                continue;
            }
            float TTC = range / range_speed;
            if (0 < TTC && TTC < 1) {
                RCLCPP_INFO(this->get_logger(), "Braking %f", TTC);
                auto ackermann_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
                ackermann_msg->drive.speed = 0.0;
                drive_command_publisher_->publish(std::move(ackermann_msg));
            }
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        speed_ = odom_msg->twist.twist.linear.x;
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_command_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_car_subscription_;
    float speed_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}
