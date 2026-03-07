#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers

        sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 
            10,
            std::bind(&Safety::scan_callback, this, std::placeholders::_1));

        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, std::placeholders::_1));

        pub_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    }

private:
    double speed = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;
    /// TODO: create ROS subscribers and publishers

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        speed = msg->twist.twist.linear.x;

    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        int center_index;
        double distance_front;
        double ttc;

        center_index = scan_msg->ranges.size() / 2;
        distance_front = scan_msg->ranges[center_index];

        if (speed <= 0.01) return;
        
        ttc = distance_front/speed;
        RCLCPP_INFO(this->get_logger(), "TTC: %f", ttc);
        RCLCPP_INFO(this->get_logger(), "Speed: %f", speed);


        /// TODO: publish drive/brake message
        if (ttc<=1.5){
            auto message_drive = ackermann_msgs::msg::AckermannDriveStamped();
            message_drive.drive.speed = 0.0;
            pub_drive->publish(message_drive);
            RCLCPP_WARN(this->get_logger(), "EMERGENCY BRAKE APPLIED!");
        }
    }



};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}