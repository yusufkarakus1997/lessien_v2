// lessien_wheel_odometry: Pico B measured RPM -> /wheel_odom (nav_msgs/Odometry).
// NO TF publisher on odom->base_footprint (EKF owns that edge later).
// Frame conventions (REP-103): +x forward, +y left, +z up, +wz CCW / left turn.

#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace lessien_odometry
{

static constexpr int8_t MOTOR_MEASURED_DIRECTION_SIGN[4] = {
    +1,   // ID1 FL: +RPM = physical forward
    +1,   // ID2 RL: +RPM = physical forward
    -1,   // ID3 FR: -RPM = physical forward (right-side wiring polarity verified)
    -1    // ID4 RR: -RPM = physical forward
};
static_assert(MOTOR_MEASURED_DIRECTION_SIGN[0] == +1, "FL sign must remain +1");
static_assert(MOTOR_MEASURED_DIRECTION_SIGN[1] == +1, "RL sign must remain +1");
static_assert(MOTOR_MEASURED_DIRECTION_SIGN[2] == -1, "FR sign must remain -1 (reverse wiring)");
static_assert(MOTOR_MEASURED_DIRECTION_SIGN[3] == -1, "RR sign must remain -1 (reverse wiring)");

static constexpr double TWO_PI = 2.0 * M_PI;
static constexpr double RPM_TO_RAD_PER_S = TWO_PI / 60.0;
static constexpr size_t MOTOR_COUNT = 4;
static constexpr size_t MOTOR_FL = 0;
static constexpr size_t MOTOR_RL = 1;
static constexpr size_t MOTOR_FR = 2;
static constexpr size_t MOTOR_RR = 3;

class WheelOdometryNode : public rclcpp::Node
{
public:
  WheelOdometryNode()
  : Node("lessien_wheel_odometry",
         rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // ----- Parameters ----------------------------------------------------------
    // Geometry single-source-of-truth is lessien_description/config/geometry.yaml
    // injected at launch time; these are only ROS-side defaults / fallbacks.
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.05035);
    track_width_ = declare_parameter<double>("track_width", 0.145);
    rpm_zero_threshold_ = declare_parameter<double>("rpm_zero_threshold", 1.0);
    rpm_filter_window_ = declare_parameter<int>("rpm_filter_window", 5);
    max_dt_ = declare_parameter<double>("max_dt", 0.25);
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
    const auto rpm_topic = declare_parameter<std::string>("rpm_topic", "/pico_b/motors/rpm");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/wheel_odom");

    if (wheel_radius_ <= 0.0 || !std::isfinite(wheel_radius_)) {
      throw std::invalid_argument("wheel_radius must be positive finite");
    }
    if (track_width_ <= 0.0 || !std::isfinite(track_width_)) {
      throw std::invalid_argument("track_width must be positive finite");
    }
    if (rpm_zero_threshold_ < 0.0 || !std::isfinite(rpm_zero_threshold_)) {
      throw std::invalid_argument("rpm_zero_threshold must be finite >= 0");
    }
    if (rpm_filter_window_ < 1) {
      throw std::invalid_argument("rpm_filter_window must be >= 1 (1 disables filtering)");
    }
    if (max_dt_ <= 0.0 || !std::isfinite(max_dt_)) {
      throw std::invalid_argument("max_dt must be positive finite");
    }

    RCLCPP_INFO(get_logger(),
                "lessien_wheel_odometry: r=%.5fm track=%.4fm rpm_dead=%.2f rpm_window=%d max_dt=%.3f",
                wheel_radius_, track_width_, rpm_zero_threshold_, rpm_filter_window_, max_dt_);
    RCLCPP_INFO(get_logger(), "  rpm_topic=%s  odom_topic=%s  frames=(%s->%s)",
                rpm_topic.c_str(), odom_topic.c_str(),
                odom_frame_id_.c_str(), base_frame_id_.c_str());
    if (rpm_filter_window_ == 1) {
      RCLCPP_INFO(get_logger(), "  rpm_filter_window=1 -> RPM moving-average disabled.");
    } else {
      RCLCPP_INFO(get_logger(),
                  "  rpm_filter_window=%d -> N-sample moving avg per motor (no wait at startup).",
                  rpm_filter_window_);
    }

    // ----- Publishers / Subscribers -------------------------------------------
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    rpm_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      rpm_topic, rclcpp::SensorDataQoS().keep_last(20),
      std::bind(&WheelOdometryNode::onRpm, this, std::placeholders::_1));
  }

private:
  void onRpm(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    // Production Pico B RPM message canonical layout (4-element Int16MultiArray):
    //   idx 0: FL measured RPM (raw, sign-correction: *MOTOR_MEASURED_DIRECTION_SIGN[FL])
    //   idx 1: RL measured RPM (raw)
    //   idx 2: FR measured RPM (raw)
    //   idx 3: RR measured RPM (raw)
    // Real runtime evidence: ros2 topic echo /pico_b/motors/rpm --once
    //   data: [0, 0, 0, 0]  (4 elements)
    // Source evidence: Pico B firmware
    //   MOTOR_COUNT = 4 (L139)
    //   initMotorsRpmMessage: data.size = MOTOR_COUNT = 4 (L908)
    //   publishMotorsRpm: fills data[0..3] = measuredRpm[FL,RL,FR,RR] (L2688-2691)
    //   MOTOR_CONFIGS[4] = { {1,"FL"}, {2,"RL"}, {3,"FR"}, {4,"RR"} } (L563-567)
    if (msg->data.size() != MOTOR_COUNT) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000ULL,
        "Rejecting RPM sample: expected %zu elements (production 4-layout), got %zu",
        MOTOR_COUNT, msg->data.size());
      return;
    }

    // Pipeline order (per-motor):
    //   1. raw RPM (msg->data[i], int16) -> indices 0..3
    //   2. motor direction sign correction  -> physical signed RPM (float)
    //   3. N-sample moving average          -> smoothed physical RPM
    //   4. rpm_zero_threshold deadband      -> final used RPM (per motor)
    // Then:
    //   side averages (left/right) -> vl/vr -> vx/wz -> midpoint integration
    double physical[MOTOR_COUNT];
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      // Step 2: sign correction (physical-forward-positive convention)
      // Sign correction applied EXACTLY ONCE (raw measured -> physical forward-positive).
      physical[i] = static_cast<double>(static_cast<int>(msg->data[i])) *
                    static_cast<double>(MOTOR_MEASURED_DIRECTION_SIGN[i]);
    }

    double filtered[MOTOR_COUNT];
    if (rpm_filter_window_ <= 1) {
      // Filtering disabled: identity pass-through
      for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        filtered[i] = physical[i];
      }
    } else {
      // Step 3: per-motor variable-length moving average.
      // We do NOT wait until the buffer is full; use the current fill count
      // as the divisor so the first sample contributes immediately.
      for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        auto & buf = rpm_buffer_[i];
        buf.push_back(physical[i]);
        if (buf.size() > static_cast<size_t>(rpm_filter_window_)) {
          buf.pop_front();
        }
        double sum = 0.0;
        for (const auto v : buf) {
          sum += v;
        }
        filtered[i] = sum / static_cast<double>(buf.size());
      }
    }

    // Step 4: rpm_zero_threshold deadband (applied AFTER smoothing).
    double used[MOTOR_COUNT];
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      used[i] = (std::fabs(filtered[i]) <= rpm_zero_threshold_) ? 0.0 : filtered[i];
    }

    const double fl = used[MOTOR_FL];
    const double rl = used[MOTOR_RL];
    const double fr = used[MOTOR_FR];
    const double rr = used[MOTOR_RR];

    // Side averages (skid-steer 4-wheel simplified to differential-steer)
    const double left_rpm  = 0.5 * (fl + rl);
    const double right_rpm = 0.5 * (fr + rr);

    // Angular velocity (rad/s) at each side
    const double omega_left  = left_rpm  * RPM_TO_RAD_PER_S;
    const double omega_right = right_rpm * RPM_TO_RAD_PER_S;

    // Linear tangential velocity at each wheel contact patch
    const double v_left  = omega_left  * wheel_radius_;
    const double v_right = omega_right * wheel_radius_;

    // Robot body-frame twist (REP-103)
    const double vx = 0.5 * (v_left + v_right);
    const double wz = (v_right - v_left) / track_width_;

    // --- Timestamp + dt guard ------------------------------------------------
    // std_msgs/msg/Int16MultiArray intentionally has no header/stamp; use ROS
    // arrival time as the proxy sample time (micro-ROS publish is synchronous).
    const rclcpp::Time stamp = now();
    const double dt = last_stamp_.nanoseconds() == 0
      ? 0.0
      : (stamp - last_stamp_).seconds();
    last_stamp_ = stamp;

    // Integrate pose only after first sample; skip pathological deltas
    bool integrate = (dt > 0.0) && (dt <= max_dt_);
    if (!integrate && last_stamp_.nanoseconds() != 0 && dt != 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000ULL,
        "Skipping pose integration: dt=%.4f (max_dt=%.3f); dt<=0 or stale.", dt, max_dt_);
    }
    if (integrate) {
      const double yaw_mid = yaw_ + 0.5 * wz * dt;
      x_   += vx * std::cos(yaw_mid) * dt;
      y_   += vx * std::sin(yaw_mid) * dt;
      yaw_ += wz * dt;
      // Normalize yaw to [-pi, pi]
      yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));
    }

    // --- Build Odometry output ------------------------------------------------
    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = odom_frame_id_;
    out.child_frame_id = base_frame_id_;

    out.pose.pose.position.x = x_;
    out.pose.pose.position.y = y_;
    out.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    out.pose.pose.orientation = tf2::toMsg(q);

    out.twist.twist.linear.x = vx;
    out.twist.twist.linear.y = 0.0;
    out.twist.twist.linear.z = 0.0;
    out.twist.twist.angular.x = 0.0;
    out.twist.twist.angular.y = 0.0;
    out.twist.twist.angular.z = wz;

    // Covariances: conservative INITIAL TUNING values (not measured noise).
    // EKF v1 will primarily fuse twist(linear.x, angular.z) from wheel odom
    // together with orientation/angular_velocity from the IMU.
    //
    // Pose covariance (integrated, so drifts unbounded; inflated vs twist).
    const double pose_xy_var = 0.10;   // 10cm std-dev placeholder
    const double pose_yaw_var = 0.10;  // ~18deg placeholder
    std::fill(out.pose.covariance.begin(), out.pose.covariance.end(), 0.0);
    out.pose.covariance[0 * 6 + 0] = pose_xy_var * pose_xy_var;    // xx
    out.pose.covariance[1 * 6 + 1] = pose_xy_var * pose_xy_var;    // yy
    out.pose.covariance[5 * 6 + 5] = pose_yaw_var * pose_yaw_var;  // yawyaw
    out.pose.covariance[2 * 6 + 2] = 1e-3;    // zz (fixed 2D plane; small)
    out.pose.covariance[3 * 6 + 3] = 1e-3;    // roll
    out.pose.covariance[4 * 6 + 4] = 1e-3;    // pitch

    // Twist covariance (per-sample, in base_footprint frame).
    const double vx_var = 0.02;   // 2cm/s std-dev
    const double wz_var = 0.04;   // ~2.3deg/s std-dev
    const double vy_stddev = 0.05;  // 5cm/s lateral non-holonomic placeholder
    std::fill(out.twist.covariance.begin(), out.twist.covariance.end(), 0.0);
    out.twist.covariance[0 * 6 + 0] = vx_var * vx_var;
    out.twist.covariance[1 * 6 + 1] = vy_stddev * vy_stddev;
    out.twist.covariance[5 * 6 + 5] = wz_var * wz_var;
    out.twist.covariance[2 * 6 + 2] = 1e-3;
    out.twist.covariance[3 * 6 + 3] = 1e-3;
    out.twist.covariance[4 * 6 + 4] = 1e-3;

    odom_pub_->publish(out);
  }

  // Parameters
  double wheel_radius_;
  double track_width_;
  double rpm_zero_threshold_;
  int rpm_filter_window_;
  double max_dt_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  // IO
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rpm_sub_;

  // Per-motor rolling buffers for moving-average RPM smoothing.
  // rpm_buffer_[i] stores the most recent (up to rpm_filter_window_) signed-physical RPM samples.
  // At startup we do not wait for a full window; divisor is the current fill size.
  std::deque<double> rpm_buffer_[MOTOR_COUNT];

  // State
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  double x_   = 0.0;
  double y_   = 0.0;
  double yaw_ = 0.0;
};

}  // namespace lessien_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<lessien_odometry::WheelOdometryNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "lessien_wheel_odometry fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
