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
        subOdometry = this->create_subscription<Odometry>("/ego_racecar/odom", 1, std::bind(&Safety::drive_callback, this, _1));
        subLaserScan = this->create_subscription<LaserScan>("/scan", 1, std::bind(&Safety::scan_callback, this, _1));

        speed = 0.0;
    }

private:
    double speed = 0.0;
    const double MAGIC_NUMBER = 3.1;
    const double DECCELERATION = 8.26;

    Publisher<AckermannDriveStamped>::SharedPtr pubBrake;
    Subscription<Odometry>::SharedPtr subOdometry;
    Subscription<LaserScan>::SharedPtr subLaserScan;

    void drive_callback(const Odometry::ConstSharedPtr msg)
    {
        speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const LaserScan::ConstSharedPtr scanMsg)
    {
        double currentAngleRad = scanMsg->angle_min - scanMsg->angle_increment;
        double secondsNeededToBrake = std::max(MAGIC_NUMBER * abs(speed) / DECCELERATION, 0.4);
        // RCLCPP_INFO(this->get_logger(), "secondsNeededToBrake : " + std::to_string(secondsNeededToBrake));

        double minTtc = std::numeric_limits<double>::infinity();
        for (const float distance : scanMsg->ranges)
        {
            currentAngleRad += scanMsg->angle_increment;

            if (std::isnan(distance) || std::isinf(distance))
                continue;

            double projectedSpeed = speed * cos(currentAngleRad);
            if (projectedSpeed <= 0)
                continue;

            double secondsBeforeCollision = distance / projectedSpeed;

            // This is only for debugging purposes
            if (secondsBeforeCollision < minTtc)
                minTtc = secondsBeforeCollision;

            if (secondsBeforeCollision < secondsNeededToBrake)
            {
                float currentAngleDegree = currentAngleRad * 180.0 / M_PI;
                RCLCPP_INFO(this->get_logger(), "BRAKING at angle " + std::to_string(currentAngleDegree));
                brake();
                break;
            }
        }

        // RCLCPP_INFO(this->get_logger(), "minTtc : " + std::to_string(minTtc));
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