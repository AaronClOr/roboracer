#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
 
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
 
// ─────────────────────────────────────────────
//  Tunable parameters – adjust for your car/sim
// ─────────────────────────────────────────────
static constexpr double kMaxLidarRange      = 10.0;   // [m]  clip far returns
static constexpr double kBubbleRadius       = 0.35;   // [m]  safety bubble radius
static constexpr double kMinGapWidth        = 0.5;    // [m]  ignore tiny gaps
static constexpr double kMaxSpeed           = 6.0;    // [m/s]
static constexpr double kMinSpeed           = 1.5;    // [m/s]
static constexpr double kStraightThreshold  = 0.08;   // [rad] ~5 deg → "straight"
static constexpr double kMidThreshold       = 0.25;   // [rad] ~14 deg
static constexpr double kStraightSpeed      = kMaxSpeed;
static constexpr double kMidSpeed           = 3.5;    // [m/s]
static constexpr double kTurnSpeed          = kMinSpeed;
 
// ─────────────────────────────────────────────
 
class FollowTheGap : public rclcpp::Node
{
public:
  FollowTheGap() : Node("follow_the_gap")
  {
    // QoS: best-effort sensor data
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .best_effort()
                   .durability_volatile();
 
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos,
      std::bind(&FollowTheGap::scanCallback, this, std::placeholders::_1));
 
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10);
 
    RCLCPP_INFO(get_logger(), "Follow-the-Gap node started.");
  }
 
private:
  // ── ROS handles ──────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
 
  // ── Main callback ────────────────────────────
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 1. Pre-process ranges
    std::vector<float> ranges = preprocessRanges(msg->ranges);
    const int N = static_cast<int>(ranges.size());
 
    // 2. Find the closest point index
    int closest_idx = findClosest(ranges);
 
    // 3. Draw safety bubble (zero out points within bubble radius)
    drawBubble(ranges, closest_idx, msg->angle_increment);
 
    // 4. Find the best gap (largest consecutive non-zero window)
    auto [gap_start, gap_end] = findBestGap(ranges);
 
    if (gap_start == -1) {
      RCLCPP_WARN(get_logger(), "No valid gap found – stopping.");
      publishDrive(0.0, kMinSpeed * 0.5);
      return;
    }
 
    // 5. Choose best goal point ("Better Idea": mid of the furthest sub-window)
    int goal_idx = findGoalPoint(ranges, gap_start, gap_end);
 
    // 6. Convert goal index to steering angle and publish
    //    angle = angle_min + goal_idx * angle_increment
    double steering_angle =
        msg->angle_min + goal_idx * static_cast<double>(msg->angle_increment);
 
    // Clamp to physical limits (typical F1TENTH: ±0.4189 rad ≈ ±24°)
    steering_angle = std::clamp(steering_angle, -0.4189, 0.4189);
 
    // Speed schedule: go slower when turning
    double speed = speedFromSteering(steering_angle);
 
    publishDrive(steering_angle, speed);
 
    RCLCPP_DEBUG(get_logger(),
                 "closest=%d  gap=[%d,%d]  goal=%d  steer=%.3f  speed=%.2f",
                 closest_idx, gap_start, gap_end, goal_idx, steering_angle, speed);
  }
 
  // ── Step 1: Pre-process ───────────────────────
  std::vector<float> preprocessRanges(const std::vector<float>& raw) const
  {
    std::vector<float> r(raw.size());
    for (size_t i = 0; i < raw.size(); ++i) {
      float v = raw[i];
      // Replace NaN/Inf and out-of-range values
      if (!std::isfinite(v) || v <= 0.0f)
        r[i] = 0.0f;
      else
        r[i] = std::min(v, static_cast<float>(kMaxLidarRange));
    }
    // Optional: smooth with a small moving average to reduce noise
    smoothRanges(r);
    return r;
  }
 
  void smoothRanges(std::vector<float>& r) const
  {
    const int K = 3; // half-window
    std::vector<float> tmp(r.size(), 0.0f);
    for (int i = 0; i < static_cast<int>(r.size()); ++i) {
      float sum = 0.0f;
      int   cnt = 0;
      for (int j = i - K; j <= i + K; ++j) {
        if (j >= 0 && j < static_cast<int>(r.size())) {
          sum += r[j];
          ++cnt;
        }
      }
      tmp[i] = (cnt > 0) ? sum / cnt : 0.0f;
    }
    r = tmp;
  }
 
  // ── Step 2: Find closest point ───────────────
  int findClosest(const std::vector<float>& r) const
  {
    int   idx = 0;
    float mn  = std::numeric_limits<float>::max();
    for (int i = 0; i < static_cast<int>(r.size()); ++i) {
      if (r[i] > 0.0f && r[i] < mn) {
        mn  = r[i];
        idx = i;
      }
    }
    return idx;
  }
 
  // ── Step 3: Safety bubble ────────────────────
  void drawBubble(std::vector<float>& r, int center, float angle_inc) const
  {
    if (r[center] <= 0.0f) return;
    float dist        = r[center];
    // angular width subtended by the bubble at that distance
    float half_angle  = std::atan2(static_cast<float>(kBubbleRadius), dist);
    int   half_pts    = static_cast<int>(std::ceil(half_angle / angle_inc));
 
    int lo = std::max(0, center - half_pts);
    int hi = std::min(static_cast<int>(r.size()) - 1, center + half_pts);
    for (int i = lo; i <= hi; ++i)
      r[i] = 0.0f;
  }
 
  // ── Step 4: Find the best (longest) gap ──────
  std::pair<int,int> findBestGap(const std::vector<float>& r) const
  {
    int best_start = -1, best_end = -1, best_len = 0;
    int cur_start  = -1;
 
    for (int i = 0; i <= static_cast<int>(r.size()); ++i) {
      bool nonzero = (i < static_cast<int>(r.size())) && (r[i] > 0.0f);
      if (nonzero) {
        if (cur_start == -1) cur_start = i;
      } else {
        if (cur_start != -1) {
          int len = i - cur_start;
          if (len > best_len) {
            best_len   = len;
            best_start = cur_start;
            best_end   = i - 1;
          }
          cur_start = -1;
        }
      }
    }
    return {best_start, best_end};
  }
 
  // ── Step 5: Find goal point ("Better Idea") ──
  //
  // "Better Idea" from lecture: within the best gap, find the sub-window of
  // fixed width centered on the furthest point, then take the midpoint of that
  // sub-window as the steering target.  This keeps the car away from gap edges.
  int findGoalPoint(const std::vector<float>& r, int gap_start, int gap_end) const
  {
    // Find the furthest point inside the gap
    int   far_idx = gap_start;
    float far_val = 0.0f;
    for (int i = gap_start; i <= gap_end; ++i) {
      if (r[i] > far_val) {
        far_val = r[i];
        far_idx = i;
      }
    }
 
    // Return the midpoint of the gap (robust to near-edge furthest points)
    // Blend: weight towards far_idx but stay within the gap center
    int mid = (gap_start + gap_end) / 2;
    // Weighted blend: 60% furthest, 40% midpoint
    int goal = static_cast<int>(std::round(0.6 * far_idx + 0.4 * mid));
    goal = std::clamp(goal, gap_start, gap_end);
    return goal;
  }
 
  // ── Step 6: Speed schedule ───────────────────
  double speedFromSteering(double steer) const
  {
    double abs_steer = std::abs(steer);
    if (abs_steer < kStraightThreshold)
      return kStraightSpeed;
    else if (abs_steer < kMidThreshold)
      return kMidSpeed;
    else
      return kTurnSpeed;
  }
 
  // ── Publish drive command ────────────────────
  void publishDrive(double steering_angle, double speed) const
  {
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp    = now();
    msg.header.frame_id = "base_link";
    msg.drive.steering_angle = static_cast<float>(steering_angle);
    msg.drive.speed          = static_cast<float>(speed);
    drive_pub_->publish(msg);
  }
};
 
// ─────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowTheGap>());
  rclcpp::shutdown();
  return 0;
}