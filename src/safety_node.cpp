#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace rclcpp;
using std::placeholders::_1;

typedef ackermann_msgs::msg::AckermannDriveStamped AckermannDriveStamped;
typedef nav_msgs::msg::Odometry Odometry;
typedef sensor_msgs::msg::LaserScan LaserScan;

class Safety : public rclcpp::Node
{
public:
    Safety() : Node("safety_node")
    {
        pubBrake = this->create_publisher<AckermannDriveStamped>("/drive", 1);
        subLaserScan = this->create_subscription<LaserScan>("/scan", 1, std::bind(&Safety::scan_callback, this, _1));

        lastScanMsg = nullptr;
    }

private:
    LaserScan::ConstSharedPtr lastScanMsg;
    // const double MAGIC_NUMBER = 3.1;
    // const double DECCELERATION = 8.26;

    Publisher<AckermannDriveStamped>::SharedPtr pubBrake;
    Subscription<LaserScan>::SharedPtr subLaserScan;

    void scan_callback(const LaserScan::ConstSharedPtr scanMsg)
    {
        if (lastScanMsg == nullptr || scanMsg->ranges.size() != lastScanMsg->ranges.size())
        {
            lastScanMsg = scanMsg;
            return;
        }

        float timeNeededToBrake = 0.1; // std::max(MAGIC_NUMBER * abs(speed) / DECCELERATION, 0.4);
        double scanTimeDiffSec = (double)(scanMsg->header.stamp.nanosec - lastScanMsg->header.stamp.nanosec) / 1000000000.0;
        RCLCPP_INFO(this->get_logger(), "scanTimeDiffSec : " + std::to_string(scanTimeDiffSec));

        float minTtc = std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < scanMsg->ranges.size(); i++)
        {
            float distanceNow = scanMsg->ranges[i];
            float distanceBefore = lastScanMsg->ranges[i];

            if (std::isnan(distanceNow) || std::isinf(distanceNow) ||
                std::isnan(distanceBefore) || std::isinf(distanceBefore))
                continue;

            float deltaDistance = distanceNow - distanceBefore;
            if (deltaDistance >= 0) // If positive, wall is further away, no need to consider it
                continue;

            float instantSpeed = -deltaDistance / scanTimeDiffSec;
            float ttc = distanceNow / instantSpeed;
            // RCLCPP_INFO(this->get_logger(), "instantSpeed : " + std::to_string(instantSpeed));
            // RCLCPP_INFO(this->get_logger(), "deltaDistance : " + std::to_string(deltaDistance));

            // This is only for debugging purposes
            if (ttc < minTtc)
                minTtc = ttc;

            if (ttc < timeNeededToBrake)
            {
                RCLCPP_INFO(this->get_logger(), "BRAKING at ttc " + std::to_string(ttc));
                brake();
                break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "minTtc : " + std::to_string(minTtc));
        lastScanMsg = scanMsg;
    }

    void brake()
    {
        AckermannDriveStamped brakeMsg;
        brakeMsg.drive.speed = 0;

        pubBrake->publish(brakeMsg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}