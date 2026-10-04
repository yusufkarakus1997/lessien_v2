#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <cmath>
#include <chrono>
#include <string>
#include <sstream>

class EkfTripDistanceNode : public rclcpp::Node
{
public:
  EkfTripDistanceNode()
  : Node("ekf_trip_distance_node"),
    distance_m_(0.0),
    initialized_(false),
    previous_stamp_(0, 0),
    trip_reset_armed_(false),
    trip_reset_count_(0),
    trip_reset_hold_started_(false),
    trip_reset_hold_start_ms_(0),
    trip_reset_ch10_raw_last_(0),
    trip_reset_button_high_(false),
    first_low_seen_(false)
  {
    declare_parameter<double>("max_dt_sec", 0.5);
    declare_parameter<double>("deadband_vx_mps", 0.0);
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("distance_topic", "/lessien/ekf_distance_m");

    declare_parameter<bool>("trip_reset_enabled", true);
    declare_parameter<std::string>("trip_reset_sbus_topic", "/pico_b/sbus/channels");
    declare_parameter<int64_t>("trip_reset_channel", 10);
    declare_parameter<int64_t>("trip_reset_low_max", 700);
    declare_parameter<int64_t>("trip_reset_high_min", 1400);
    declare_parameter<int64_t>("trip_reset_hold_ms", 500);
    declare_parameter<std::string>("trip_reset_diagnostics_topic", "/diagnostics");

    get_parameter("max_dt_sec", max_dt_sec_);
    get_parameter("deadband_vx_mps", deadband_vx_mps_);
    get_parameter("odom_topic", odom_topic_);
    get_parameter("distance_topic", distance_topic_);

    get_parameter("trip_reset_enabled", trip_reset_enabled_);
    get_parameter("trip_reset_sbus_topic", trip_reset_sbus_topic_);
    {
      int64_t ch1based = 10;
      get_parameter("trip_reset_channel", ch1based);
      trip_reset_channel_1based_ = static_cast<int>(ch1based);
      trip_reset_channel_idx_ = trip_reset_channel_1based_ - 1;
    }
    {
      int64_t v = 700;
      get_parameter("trip_reset_low_max", v);
      trip_reset_low_max_ = static_cast<int>(v);
    }
    {
      int64_t v = 1400;
      get_parameter("trip_reset_high_min", v);
      trip_reset_high_min_ = static_cast<int>(v);
    }
    {
      int64_t v = 500;
      get_parameter("trip_reset_hold_ms", v);
      trip_reset_hold_ms_ = static_cast<int>(v);
    }
    get_parameter("trip_reset_diagnostics_topic", trip_reset_diagnostics_topic_);

    RCLCPP_INFO(this->get_logger(),
      "ekf_trip_distance_node: max_dt=%.3fs, deadband_vx=%.5f m/s",
      max_dt_sec_, deadband_vx_mps_);
    RCLCPP_INFO(this->get_logger(),
      "trip_reset: enabled=%d channel=CH%d(idx=%d) low_max=%d high_min=%d hold_ms=%d",
      trip_reset_enabled_ ? 1 : 0,
      trip_reset_channel_1based_, trip_reset_channel_idx_,
      trip_reset_low_max_, trip_reset_high_min_, trip_reset_hold_ms_);

    using std::placeholders::_1;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&EkfTripDistanceNode::odomCallback, this, _1));

    dist_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      distance_topic_,
      rclcpp::QoS(1).reliable());

    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      trip_reset_diagnostics_topic_,
      rclcpp::QoS(1).reliable());

    if (trip_reset_enabled_) {
      sbus_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        trip_reset_sbus_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&EkfTripDistanceNode::sbusCallback, this, _1));
      diag_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&EkfTripDistanceNode::publishDiagnostics, this));
    }
  }

private:
  int64_t now_ms() const {
    return static_cast<int64_t>(this->now().nanoseconds() / 1000000ULL);
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->now();
    arr.header.frame_id = "";
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.name = "lessien/ekf_trip_reset";
    st.message = trip_reset_enabled_ ? "OK" : "DISABLED";
    st.hardware_id = "pi5_lessien";

    auto addKV = [&](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = v;
      st.values.push_back(kv);
    };
    auto addInt = [&](const std::string & k, long long v) {
      std::ostringstream os; os << v; addKV(k, os.str());
    };
    auto addBool = [&](const std::string & k, bool v) { addKV(k, v ? "true" : "false"); };
    auto addDbl = [&](const std::string & k, double v, int p=3) {
      std::ostringstream os; os.precision(p); os << std::fixed << v; addKV(k, os.str());
    };

    addBool("tripResetEnabled", trip_reset_enabled_);
    addBool("tripResetArmed", trip_reset_armed_);
    addInt("tripResetCount", static_cast<long long>(trip_reset_count_));
    addInt("tripResetCh10Raw", static_cast<long long>(trip_reset_ch10_raw_last_));
    {
      int64_t hold_ms = 0;
      if (trip_reset_hold_started_) {
        hold_ms = std::max<int64_t>(0, now_ms() - trip_reset_hold_start_ms_);
      }
      trip_reset_hold_ms_last_ = static_cast<int>(hold_ms);
      addInt("tripResetHoldMs", static_cast<long long>(hold_ms));
    }
    addBool("tripResetButtonHigh", trip_reset_button_high_);
    addDbl("distanceM", distance_m_, 4);
    addBool("firstLowSeen", first_low_seen_);
    addInt("channelIdx", static_cast<long long>(trip_reset_channel_idx_));

    arr.status.push_back(st);
    diag_pub_->publish(arr);
  }

  void sbusCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (!trip_reset_enabled_) return;

    const int idx = trip_reset_channel_idx_;
    const int data_size = static_cast<int>(msg->data.size());
    if (idx < 0 || idx >= data_size) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "SBUS array size=%d but trip reset channel idx=%d out of range; skip reset.",
        data_size, idx);
      return;
    }

    const int raw = static_cast<int>(msg->data[idx]);
    trip_reset_ch10_raw_last_ = raw;

    if (raw <= trip_reset_low_max_) {
      trip_reset_button_high_ = false;
      trip_reset_hold_started_ = false;
      trip_reset_hold_start_ms_ = 0;
      first_low_seen_ = true;
      trip_reset_armed_ = true;
      return;
    }

    if (raw >= trip_reset_high_min_) {
      trip_reset_button_high_ = true;
      if (!trip_reset_armed_ || !first_low_seen_) {
        trip_reset_hold_started_ = false;
        return;
      }
      const int64_t tnow = now_ms();
      if (!trip_reset_hold_started_) {
        trip_reset_hold_started_ = true;
        trip_reset_hold_start_ms_ = tnow;
        return;
      }
      const int64_t elapsed = tnow - trip_reset_hold_start_ms_;
      if (elapsed >= static_cast<int64_t>(trip_reset_hold_ms_)) {
        performTripReset();
        trip_reset_armed_ = false;
        trip_reset_hold_started_ = false;
        trip_reset_hold_start_ms_ = 0;
      }
      return;
    }

    trip_reset_button_high_ = false;
    trip_reset_hold_started_ = false;
    trip_reset_hold_start_ms_ = 0;
  }

  void performTripReset()
  {
    distance_m_ = 0.0;
    initialized_ = false;
    previous_stamp_ = rclcpp::Time(0, 0);
    ++trip_reset_count_;
    RCLCPP_INFO(this->get_logger(),
      "CH%d TRIP RESET -> distance=0.0m count=%lu (armed=false until next LOW)",
      trip_reset_channel_1based_,
      static_cast<unsigned long>(trip_reset_count_));
    publishDistance();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double vx = msg->twist.twist.linear.x;
    const rclcpp::Time stamp(msg->header.stamp.sec, msg->header.stamp.nanosec);

    if (!initialized_) {
      initialized_ = true;
      previous_stamp_ = stamp;
      publishDistance();
      return;
    }

    const double dt = (stamp - previous_stamp_).seconds();

    if (dt <= 0.0 || dt > max_dt_sec_) {
      previous_stamp_ = stamp;
      publishDistance();
      return;
    }

    if (!std::isfinite(vx) || !std::isfinite(dt)) {
      previous_stamp_ = stamp;
      publishDistance();
      return;
    }

    double vx_effective = vx;
    if (std::fabs(vx_effective) < deadband_vx_mps_) {
      vx_effective = 0.0;
    }

    const double increment = vx_effective * dt;
    if (std::isfinite(increment)) {
      distance_m_ += increment;
    }

    previous_stamp_ = stamp;
    publishDistance();
  }

  void publishDistance()
  {
    auto out = std_msgs::msg::Float32();
    if (std::isfinite(distance_m_)) {
      out.data = static_cast<float>(distance_m_);
    } else {
      out.data = 0.0f;
      RCLCPP_WARN(this->get_logger(), "distance_m_ NaN/Inf, reset to 0");
      distance_m_ = 0.0;
    }
    dist_pub_->publish(out);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sbus_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  double distance_m_;
  bool initialized_;
  rclcpp::Time previous_stamp_;

  double max_dt_sec_;
  double deadband_vx_mps_;
  std::string odom_topic_;
  std::string distance_topic_;

  bool trip_reset_enabled_;
  std::string trip_reset_sbus_topic_;
  int trip_reset_channel_1based_;
  int trip_reset_channel_idx_;
  int trip_reset_low_max_;
  int trip_reset_high_min_;
  int trip_reset_hold_ms_;
  std::string trip_reset_diagnostics_topic_;

  bool trip_reset_armed_;
  uint64_t trip_reset_count_;
  bool trip_reset_hold_started_;
  int64_t trip_reset_hold_start_ms_;
  int trip_reset_ch10_raw_last_;
  bool trip_reset_button_high_;
  mutable int trip_reset_hold_ms_last_;
  bool first_low_seen_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfTripDistanceNode>());
  rclcpp::shutdown();
  return 0;
}
