#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

class WallController : public rclcpp::Node
{
public:
  WallController() : Node("wall_controller"),
    prev_error_(0.0), integral_(0.0), prev_time_(this->now())
  {
    this->declare_parameter<double>("desired_distance", 0.7);
    this->declare_parameter<double>("kp", 0.85);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.1);

    steering_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "/autodrive/roboracer_1/steering_command", 10);
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "/autodrive/roboracer_1/throttle_command_raw", 10);

    wall_data_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/wall_data", 10,
      std::bind(&WallController::wall_data_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "WallController node started.");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wall_data_sub_;

  double prev_error_;
  double integral_;
  rclcpp::Time prev_time_;

  void wall_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    double D_t1   = msg->data[0];
    //double alpha  = msg->data[1];
     RCLCPP_INFO(this->get_logger(), "received thingy.");

    double desired = this->get_parameter("desired_distance").as_double();
    double kp      = this->get_parameter("kp").as_double();
    double ki      = this->get_parameter("ki").as_double();
    double kd      = this->get_parameter("kd").as_double();

    // --- Step 4: PID ---
    double error = desired - D_t1;

    rclcpp::Time now = this->now();
    double dt = (now - prev_time_).seconds();
    prev_time_ = now;

    if (dt <= 0.0) return; // skip first tick

    integral_  += error * dt;
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    double steering_angle = kp * error + ki * integral_ + kd * derivative;

    // --- Step 5: Speed based on steering angle ---
    double abs_angle_deg = std::abs(steering_angle) * 180.0 / M_PI;
    double speed;
    if      (abs_angle_deg <= 10.0) speed = 0.18;
    else if (abs_angle_deg <= 20.0) speed = 0.12;
    else                            speed = 0.06;

    // --- Step 6: Publish to /drive ---
    auto steering_msg = std_msgs::msg::Float32();
    auto throttle_msg = std_msgs::msg::Float32();

    steering_msg.data = static_cast<float>(steering_angle);
    throttle_msg.data = static_cast<float>(speed);

    steering_pub_->publish(steering_msg);
    throttle_pub_->publish(throttle_msg);
    RCLCPP_INFO(this->get_logger(),
      "error=%.3f | steering=%.3f rad | speed=%.1f m/s",
      error, steering_angle, speed);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallController>());
  rclcpp::shutdown();
  return 0;
}