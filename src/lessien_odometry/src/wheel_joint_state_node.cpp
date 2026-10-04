// wheel_joint_state_node: Pico B measured RPM -> /joint_states (sensor_msgs/JointState).
// Integrates per-wheel RPM into continuous wheel angle + velocity; used by
// production robot_state_publisher to animate the 4 continuous wheel-joint TF
// frames so RViz shows rotating wheels matching commanded motor motion.
// NO TF publisher on this node (RSP is sole TF owner below base_link).

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace lessien_odometry
{

static constexpr int8_t MOTOR_MEASURED_DIRECTION_SIGN[4] = {
    +1,   // ID0 FL: +RPM = physical forward (verified with wheel_odometry)
    +1,   // ID1 RL: +RPM = physical forward
    -1,   // ID2 FR: -RPM = physical forward (right-side wiring polarity)
    -1,   // ID3 RR: -RPM = physical forward
};
static_assert(MOTOR_MEASURED_DIRECTION_SIGN[0] == +1, "FL sign must remain +1");
static_assert(MOTOR_MEASURED_DIRECTION_SIGN[1] == +1, "RL sign must remain +1");
static_assert(MOTOR_MEASURED_DIRECTION_SIGN[2] == -1, "FR sign must remain -1");
static_assert(MOTOR_MEASURED_DIRECTION_SIGN[3] == -1, "RR sign must remain -1");

static constexpr double TWO_PI = 2.0 * M_PI;
static constexpr double RPM_TO_RAD_PER_S = TWO_PI / 60.0;
static constexpr size_t MOTOR_COUNT = 4;
static constexpr size_t MOTOR_FL = 0;
static constexpr size_t MOTOR_RL = 1;
static constexpr size_t MOTOR_FR = 2;
static constexpr size_t MOTOR_RR = 3;

static constexpr const char * JOINT_NAMES[MOTOR_COUNT] = {
    "front_left_wheel_joint",
    "rear_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_right_wheel_joint",
};

class WheelJointStateNode : public rclcpp::Node
{
public:
  WheelJointStateNode()
  : Node("wheel_joint_state_node",
         rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    const auto rpm_topic =
      declare_parameter<std::string>("rpm_topic", "/pico_b/motors/rpm");
    const auto joint_topic =
      declare_parameter<std::string>("joint_states_topic", "/joint_states");
    max_dt_ = declare_parameter<double>("max_dt", 0.25);
    rpm_zero_threshold_ = declare_parameter<double>("rpm_zero_threshold", 1.0);

    if (max_dt_ <= 0.0 || !std::isfinite(max_dt_)) {
      throw std::invalid_argument("max_dt must be positive finite");
    }
    if (rpm_zero_threshold_ < 0.0 || !std::isfinite(rpm_zero_threshold_)) {
      throw std::invalid_argument("rpm_zero_threshold must be finite >= 0");
    }

    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      theta_[i] = 0.0;
    }
    last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    first_sample_ = true;

    publisher_ = create_publisher<sensor_msgs::msg::JointState>(joint_topic, 10);

    subscription_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      rpm_topic, rclcpp::SensorDataQoS().keep_last(20),
      std::bind(&WheelJointStateNode::onRpm, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "wheel_joint_state_node: rpm=%s joint_states=%s max_dt=%.3fs rpm_dead=%.2f",
                rpm_topic.c_str(), joint_topic.c_str(), max_dt_, rpm_zero_threshold_);
    RCLCPP_INFO(get_logger(),
                "  joints: [FL=%s RL=%s FR=%s RR=%s]",
                JOINT_NAMES[MOTOR_FL], JOINT_NAMES[MOTOR_RL],
                JOINT_NAMES[MOTOR_FR], JOINT_NAMES[MOTOR_RR]);
    RCLCPP_INFO(get_logger(),
                "  sign correction (Pico raw -> physical forward +RPM):"
                " FL+%d RL+%d FR+%d RR+%d",
                MOTOR_MEASURED_DIRECTION_SIGN[MOTOR_FL],
                MOTOR_MEASURED_DIRECTION_SIGN[MOTOR_RL],
                MOTOR_MEASURED_DIRECTION_SIGN[MOTOR_FR],
                MOTOR_MEASURED_DIRECTION_SIGN[MOTOR_RR]);
  }

private:
  void onRpm(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    // Production Pico B RPM message canonical layout (4-element Int16MultiArray):
    //   idx 0: FL measured RPM (raw)
    //   idx 1: RL measured RPM (raw)
    //   idx 2: FR measured RPM (raw)
    //   idx 3: RR measured RPM (raw)
    // Real runtime evidence: ros2 topic echo /pico_b/motors/rpm --once -> 4 elements.
    // Pico publisher QoS: BEST_EFFORT (verified via ros2 topic info --verbose).
    if (msg->data.size() != MOTOR_COUNT) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "RPM array: expected %zu (production 4-layout), got %zu — skipping.",
                           MOTOR_COUNT, msg->data.size());
      return;
    }

    rclcpp::Time now = get_clock()->now();

    // --- Sign correction ----------------------------------------------------
    // Apply Pico-B wiring direction signs exactly once (identical to wheel_odometry).
    double physical_rpm[MOTOR_COUNT];
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      physical_rpm[i] = static_cast<double>(msg->data[i]) *
                        MOTOR_MEASURED_DIRECTION_SIGN[i];
    }

    // --- Deadband -----------------------------------------------------------
    double used_rpm[MOTOR_COUNT];
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      used_rpm[i] = (std::fabs(physical_rpm[i]) <= rpm_zero_threshold_) ? 0.0 : physical_rpm[i];
    }

    // --- Angular velocity (rad/s) -------------------------------------------
    double omega_rad_s[MOTOR_COUNT];
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      omega_rad_s[i] = used_rpm[i] * RPM_TO_RAD_PER_S;
    }

    // --- Angle integration --------------------------------------------------
    // First call: just init timestamp and skip giant dt.
    if (first_sample_) {
      first_sample_ = false;
      last_stamp_ = now;
    } else {
      double dt = (now - last_stamp_).seconds();
      if (dt > 0.0 && dt <= max_dt_) {
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
          theta_[i] += omega_rad_s[i] * dt;
        }
        // Wrapped-normalize to (-pi, pi] so the float exponent stays bounded.
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
          theta_[i] = std::atan2(std::sin(theta_[i]), std::cos(theta_[i]));
        }
        last_stamp_ = now;
      } else if (dt > max_dt_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "dt=%.3fs > max_dt=%.3fs — skipping integration, reset stamp.",
                             dt, max_dt_);
        last_stamp_ = now;
      }
      // else dt <= 0: skip, keep previous angle.
    }

    // --- Publish ------------------------------------------------------------
    auto js = std::make_unique<sensor_msgs::msg::JointState>();
    js->header.stamp = now;
    js->name.reserve(MOTOR_COUNT);
    js->position.reserve(MOTOR_COUNT);
    js->velocity.reserve(MOTOR_COUNT);
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      js->name.push_back(JOINT_NAMES[i]);
      js->position.push_back(theta_[i]);
      js->velocity.push_back(omega_rad_s[i]);
    }
    publisher_->publish(std::move(js));
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;

  double theta_[MOTOR_COUNT];
  rclcpp::Time last_stamp_;
  bool first_sample_;
  double max_dt_;
  double rpm_zero_threshold_;
};

}  // namespace lessien_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lessien_odometry::WheelJointStateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
