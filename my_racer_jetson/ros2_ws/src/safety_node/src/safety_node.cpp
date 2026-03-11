#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"


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
            //"scan", 
            "autodrive/roboracer_1/lidar",
            10,
            std::bind(&Safety::scan_callback, this, std::placeholders::_1));

        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            //"/ego_racecar/odom", 
            "autodrive/roboracer_1/odom",
            10, 
            std::bind(&Safety::drive_callback, this, std::placeholders::_1));

        // pub_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        pub_drive = this->create_publisher<std_msgs::msg::Float32>("autodrive/roboracer_1/throttle_command", 10);

        sub_thottle = this->create_subscription<std_msgs::msg::Float32>(
            //"/ego_racecar/odom", 
            "autodrive/roboracer_1/throttle_command_raw",
            10, 
            std::bind(&Safety::throttle_callback, this, std::placeholders::_1));
        // pub_brake = this->create_publisher<std_msgs::msg::Float32>("autodrive/roboracer_1/throttle", 10);
        pub_brake = this->create_publisher<std_msgs::msg::Bool>(
            //"/ego_racecar/odom", 
            "autodrive/roboracer_1/emergency_break",
            10);

    }

private:
    double speed = 0.0;
    double ttc = 10;
    double thottle = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_thottle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_drive;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_brake;

    // rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;
    /// TODO: create ROS subscribers and publishers

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        speed = msg->twist.twist.linear.x;

    }

     void throttle_callback(const std_msgs::msg::Float32::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        // auto message_drive = std_msgs::msg::Float32();
        auto message_to_publish = std_msgs::msg::Float32();
        auto e_break = std_msgs::msg::Bool();
        
        // 1. Check the safety condition determined by the scan_callback
        if (ttc <= 1.0) {
            message_to_publish.data = 0.0; // Override to Brake
            e_break.data = true;
            
            RCLCPP_WARN(this->get_logger(), "EMERGENCY BRAKE! TTC: %f", ttc);
        } else {
            message_to_publish.data = msg->data; // Pass through the user command
            e_break.data = false;

        }

        // 2. Publish only when we get a new command from the user
        pub_brake->publish(e_break);
        pub_drive->publish(message_to_publish);


    }   

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        int center_index;
        double distance_front;
        // auto message_drive = std_msgs::msg::Float32();
        //message_drive.data = thottle;
 
        center_index = scan_msg->ranges.size() / 2;
        distance_front = scan_msg->ranges[center_index];

        if (speed >= 0.01){
            // message_drive.data = 1.0;
            ttc = distance_front/speed;
            RCLCPP_INFO(this->get_logger(), "TTC: %f", ttc);
            RCLCPP_INFO(this->get_logger(), "Speed: %f", speed);
        }
        else {
            ttc = 999.9;
        }

        /// TODO: publish drive/brake message

    }



};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}