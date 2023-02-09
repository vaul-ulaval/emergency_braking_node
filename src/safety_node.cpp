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
        // All default values are set to work in the simulator
        this->declare_parameter("brakeTopicName", "/drive");
        this->declare_parameter("odomTopicName", "/ego_racecar/odom");
        this->declare_parameter("laserScanTopicName", "/scan");
        this->declare_parameter("fovDegree", 46); // FOV Must be a multiple of 2
        this->declare_parameter("speedTtcThresoldScale", 2.5);
        this->declare_parameter("carDecceleration", 8.26);

        std::string brakeTopicName = this->get_parameter("brakeTopicName").get_parameter_value().get<std::string>();
        std::string odomTopicName = this->get_parameter("odomTopicName").get_parameter_value().get<std::string>();
        std::string laserScanTopicName = this->get_parameter("laserScanTopicName").get_parameter_value().get<std::string>();
        fovDegree = this->get_parameter("fovDegree").get_parameter_value().get<int>();
        speedTtcThresoldScale = this->get_parameter("speedTtcThresoldScale").get_parameter_value().get<float>();
        carDecceleration = this->get_parameter("carDecceleration").get_parameter_value().get<float>();

        pubBrake = this->create_publisher<AckermannDriveStamped>(brakeTopicName, 1);
        subOdometry = this->create_subscription<Odometry>(odomTopicName, 1, std::bind(&Safety::drive_callback, this, _1));
        subLaserScan = this->create_subscription<LaserScan>(laserScanTopicName, 1, std::bind(&Safety::scan_callback, this, _1));

        speed = 0.0;
    }

private:
    double speed = 0.0;
    int startRangeIndex = 0;
    int endRangeIndex = 0;

    // Params
    int fovDegree;
    float speedTtcThresoldScale;
    float carDecceleration;

    // Publisher/Subscribers
    Publisher<AckermannDriveStamped>::SharedPtr pubBrake;
    Subscription<Odometry>::SharedPtr subOdometry;
    Subscription<LaserScan>::SharedPtr subLaserScan;

    void drive_callback(const Odometry::ConstSharedPtr msg)
    {
        speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const LaserScan::ConstSharedPtr scanMsg)
    {
        if (startRangeIndex == 0 && endRangeIndex == 0)
            computeRangeIndexes(scanMsg);

        double secondsNeededToBrake = 0.6; // std::max(speedTtcThresoldScale * abs(speed) / carDecceleration, 0.4);
        // RCLCPP_INFO(this->get_logger(), "secondsNeededToBrake : " + std::to_string(secondsNeededToBrake));

        double minTtc = std::numeric_limits<double>::infinity();
        for (int i = startRangeIndex; i <= endRangeIndex; i++)
        {
            float distance = scanMsg->ranges[i];
            if (std::isnan(distance) || std::isinf(distance))
                continue;

            double currentAngleRad = scanMsg->angle_min + i * scanMsg->angle_increment;

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

        RCLCPP_INFO(this->get_logger(), "minTtc : " + std::to_string(minTtc));
    }

    void brake()
    {
        AckermannDriveStamped brakeMsg;
        brakeMsg.drive.speed = 0;

        pubBrake->publish(brakeMsg);
    }

    void computeRangeIndexes(const LaserScan::ConstSharedPtr scanMsg)
    {
        float radMinAngle = (-fovDegree / 2) * (M_PI / 180.0);
        if (radMinAngle < scanMsg->angle_min)
            throw std::runtime_error("Node has an invalid fovDegree parameter");

        int index = 0;
        while (radMinAngle > scanMsg->angle_min)
        {
            radMinAngle -= scanMsg->angle_increment;
            index++;
        }

        startRangeIndex = index;
        endRangeIndex = scanMsg->ranges.size() - 1 - index;
        RCLCPP_INFO(this->get_logger(), "start : " + std::to_string(startRangeIndex));
        RCLCPP_INFO(this->get_logger(), "end : " + std::to_string(endRangeIndex));
        RCLCPP_INFO(this->get_logger(), "range : " + std::to_string(scanMsg->ranges.size()));
        RCLCPP_INFO(this->get_logger(), "increment : " + std::to_string(scanMsg->angle_increment));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}