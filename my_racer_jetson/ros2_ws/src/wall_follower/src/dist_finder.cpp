// dis_finder_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>



class DisFinder : public rclcpp::Node
{
public:
  DisFinder() : Node("dis_finder")
  {
    // Angle between beam a and beam b (in degrees, configurable)
    this->declare_parameter<double>("theta_deg", 45.0);
    this->declare_parameter<double>("lookahead", 0.7);


    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/autodrive/roboracer_1/lidar", 10,
      std::bind(&DisFinder::scan_callback, this, std::placeholders::_1));
    
    wall_data_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wall_data", 10);  
    RCLCPP_INFO(this->get_logger(), "DisFinder node started.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wall_data_pub_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double theta_deg = this->get_parameter("theta_deg").as_double();
    double L = this->get_parameter("lookahead").as_double();
    double theta = theta_deg * M_PI / 180.0;

    // --- Step 1: Obtain distances a and b ---
    //
    // beam b: 90 degrees to the right of the car's x-axis → angle = -90° = -π/2
    // beam a: θ degrees ahead of beam b               → angle = -π/2 + θ
    //
    // LiDAR angles: 0 = forward, positive = left, negative = right

    //double angle_b = -M_PI / 2.0;            // 90° right
    double angle_b = -80.0 * M_PI / 180.0;   //70 
    double angle_a = angle_b + theta;         // θ ahead of b

    double b = get_range(msg, angle_b);
    double a = get_range(msg, angle_a);

    if (std::isinf(b) || std::isinf(a) || std::isnan(b) || std::isnan(a)) {
      RCLCPP_WARN(this->get_logger(), "Invalid laser readings (inf/nan), skipping.");
      return;
    }

    // --- Step 2: Calculate angle α ---
    //
    //        a·cos(θ) - b
    // α = arctan( ------------ )
    //          a·sin(θ)

    double alpha = std::atan2(a * std::cos(theta) - b, a * std::sin(theta));
    

    RCLCPP_INFO(this->get_logger(),
      "a=%.3f m | b=%.3f m | theta=%.1f° | alpha=%.3f rad (%.1f°)",
      a, b, theta_deg, alpha, alpha * 180.0 / M_PI);

    // --- Step 3: Calculate current and future distance to wall ---
    //
    // D_t   = b · cos(α)
    // D_t+1 = D_t + L · sin(α)

    
    double D_t   = b * std::cos(alpha);
    double D_t1  = D_t + L * std::sin(alpha);

    RCLCPP_INFO(this->get_logger(),
    "a=%.3f m | b=%.3f m | alpha=%.1f° | D_t=%.3f m | D_t+1=%.3f m",
    a, b, alpha * 180.0 / M_PI, D_t, D_t1);


    // --- Step 4: send data to topic 

    auto msg_out = std_msgs::msg::Float64MultiArray();
    msg_out.data = {D_t1, alpha};
    wall_data_pub_->publish(msg_out);

  }

  // Returns the range at the desired angle by finding the closest index
  double get_range(const sensor_msgs::msg::LaserScan::SharedPtr msg, double angle)
  {
    // Clamp angle to valid scan range
    angle = std::max(static_cast<double>(msg->angle_min),
                     std::min(static_cast<double>(msg->angle_max), angle));

    int index = static_cast<int>(
      std::round((angle - msg->angle_min) / msg->angle_increment));

    // Guard against out-of-bounds
    index = std::max(0, std::min(index, static_cast<int>(msg->ranges.size()) - 1));

    return msg->ranges[index];
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisFinder>());
  rclcpp::shutdown();
  return 0;
}