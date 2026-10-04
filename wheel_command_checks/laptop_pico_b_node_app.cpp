#include <Arduino.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef PICO_B_ENABLE_OLED
#define PICO_B_ENABLE_OLED 1
#endif

#include <micro_ros_arduino.h>
#if PICO_B_ENABLE_OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif
#ifndef PICO_B_ENABLE_CAMERA_SERVO
#define PICO_B_ENABLE_CAMERA_SERVO 0
#endif
#if PICO_B_ENABLE_CAMERA_SERVO
#include <Servo.h>
#endif
#ifndef PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
#define PICO_B_ENABLE_CMD_VEL_SUBSCRIBER 0
#endif
#ifndef PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
#define PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER 0
#endif
#ifndef PICO_B_SOURCE_TIME_MAPPING_VERIFIED
#define PICO_B_SOURCE_TIME_MAPPING_VERIFIED 0
#endif
static_assert(!(PICO_B_ENABLE_CMD_VEL_SUBSCRIBER &&
                PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER),
              "Enable exactly one receive-only command subscriber");
static_assert(PICO_B_SOURCE_TIME_MAPPING_VERIFIED == 0 ||
                  PICO_B_SOURCE_TIME_MAPPING_VERIFIED == 1,
              "PICO_B_SOURCE_TIME_MAPPING_VERIFIED must be 0 or 1");
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/executor_handle.h>
#include <rclc/subscription.h>
#include <rmw_microros/ping.h>
#include <rmw_microros/time_sync.h>
#include <rmw_microros/timing.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
#include <geometry_msgs/msg/twist.h>
#endif
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
#include <lessien_interfaces/msg/wheel_command.h>
#endif

#ifndef PICO_B_ENABLE_SBUS
#define PICO_B_ENABLE_SBUS 1
#endif
#ifndef PICO_B_ENABLE_BATTERY_ADC
#define PICO_B_ENABLE_BATTERY_ADC 1
#endif
#ifndef PICO_B_ENABLE_ROS_SBUS_PUBLISHER
#define PICO_B_ENABLE_ROS_SBUS_PUBLISHER 1
#endif
#ifndef PICO_B_ENABLE_ROS_BATTERY_PUBLISHER
#define PICO_B_ENABLE_ROS_BATTERY_PUBLISHER 1
#endif
#ifndef PICO_B_ENABLE_ROS_DIAGNOSTICS
#define PICO_B_ENABLE_ROS_DIAGNOSTICS 1
#endif
#ifndef PICO_B_ENABLE_MOTORS
#define PICO_B_ENABLE_MOTORS 1
#endif
#ifndef PICO_B_ENABLE_ROS_MOTOR_PUBLISHER
#define PICO_B_ENABLE_ROS_MOTOR_PUBLISHER 1
#endif
#ifndef PICO_B_MOTOR_CYCLE_HZ
#define PICO_B_MOTOR_CYCLE_HZ 10
#endif
#ifndef PICO_B_ENABLE_SERVO
#define PICO_B_ENABLE_SERVO 0
#endif
#ifndef PICO_B_ENABLE_SPI_BRIDGE
#define PICO_B_ENABLE_SPI_BRIDGE 0
#endif
#ifndef PICO_B_ENABLE_MANUAL_OLED_PAGING
#define PICO_B_ENABLE_MANUAL_OLED_PAGING 0
#endif
#ifndef PICO_B_ENABLE_RELAYS
#define PICO_B_ENABLE_RELAYS 0
#endif

static_assert(PICO_B_ENABLE_SERVO == 0, "B3A must not attach servo");
static_assert(PICO_B_ENABLE_SPI_BRIDGE == 0, "B3A must not initialize SPI");

struct uxrCustomTransport;

constexpr uint8_t BATTERY_ADC_PIN = 26;
constexpr uint8_t SBUS_TX_PIN = 4;
constexpr uint8_t SBUS_RX_PIN = 5;
constexpr uint8_t RELAY1_PIN = 12;
constexpr uint8_t RELAY2_PIN = 13;
constexpr uint8_t RELAY_ACTIVE_LEVEL = LOW;
constexpr uint8_t RELAY_INACTIVE_LEVEL = HIGH;
constexpr uint8_t CAMERA_SERVO_SIGNAL_PIN = 6;
constexpr uint8_t RS485_BRIDGE_TX_PIN = 0;
constexpr uint8_t RS485_BRIDGE_RX_PIN = 1;
constexpr int8_t OLED_RESET_PIN = -1;
constexpr uint8_t OLED_SDA_PIN = 16;
constexpr uint8_t OLED_SCL_PIN = 17;
constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr int16_t SCREEN_WIDTH = 128;
constexpr int16_t SCREEN_HEIGHT = 64;

constexpr uint32_t HEARTBEAT_PERIOD_MS = 500;
constexpr uint32_t AGENT_PING_PERIOD_MS = 500;
constexpr uint32_t RUNNING_AGENT_PING_PERIOD_MS = 1000;
constexpr uint32_t SBUS_PUBLISH_PERIOD_MS = 50;
constexpr uint32_t BATTERY_PUBLISH_PERIOD_MS = 500;
constexpr uint32_t DIAGNOSTIC_PUBLISH_PERIOD_MS = 500;
constexpr uint32_t DIAGNOSTIC_PAGE_PERIOD_MS = 500;
constexpr uint32_t OLED_UPDATE_PERIOD_MS = 500;
constexpr uint32_t OLED_PAGE_CHANGE_PERIOD_MS = 10000;
constexpr uint32_t OLED_RATE_WINDOW_MS = 5000;
constexpr uint32_t CMD_VEL_FRESH_TIMEOUT_MS = 250;
constexpr uint32_t WHEEL_COMMAND_FRESH_TIMEOUT_MS = 250;
constexpr uint32_t WHEEL_COMMAND_SOURCE_ACCEPT_MAX_AGE_MS = 100;
constexpr uint32_t WHEEL_COMMAND_SOURCE_FUTURE_TOLERANCE_MS = 20;
constexpr uint32_t WHEEL_COMMAND_TIME_SYNC_TIMEOUT_MS = 20;
constexpr uint32_t WHEEL_COMMAND_PROTOCOL_VERSION = 1;
constexpr int AGENT_PING_TIMEOUT_MS = 20;
constexpr uint8_t RUNNING_AGENT_PING_FAILURE_LIMIT = 2;

constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
constexpr float ADC_MAX_VALUE = 4095.0f;
const float BATTERY_DIVIDER_RATIO = 9.61f;
const float BATTERY_CALIBRATION_FACTOR = 1.0f;
constexpr float BATTERY_EMPTY_VOLTAGE = 16.0f;
constexpr float BATTERY_FULL_VOLTAGE = 21.0f;
constexpr float BATTERY_LOW_VOLTAGE = 17.0f;
constexpr float BATTERY_CRITICAL_VOLTAGE = 16.3f;
constexpr float BATTERY_VALID_MIN_VOLTAGE = 10.0f;
constexpr float BATTERY_VALID_MAX_VOLTAGE = 25.0f;
constexpr float BATTERY_EMA_ALPHA = 0.15f;
constexpr uint8_t BATTERY_ADC_SAMPLE_COUNT = 16;
constexpr uint32_t BATTERY_MEASUREMENT_PERIOD_MS = 100;
constexpr size_t MOTOR_COUNT = 4;

constexpr size_t DDSM115_FRAME_SIZE = 10;
constexpr uint32_t RS485_BRIDGE_BAUD_RATE = 115200;
constexpr uint32_t RS485_RESPONSE_TIMEOUT_MS = 10;
constexpr uint32_t RS485_SPEED_GRACE_MS = 5;
constexpr uint32_t RS485_SPEED_FIRST_ATTEMPT_TOTAL_MS =
    RS485_RESPONSE_TIMEOUT_MS + RS485_SPEED_GRACE_MS;
constexpr uint32_t RS485_FRAME_QUIET_MS = 2;
constexpr uint32_t MOTOR_SLOT_PERIOD_DENOMINATOR =
    PICO_B_MOTOR_CYCLE_HZ * MOTOR_COUNT;
constexpr uint32_t MOTOR_SLOT_PERIOD_US =
    1000000UL / MOTOR_SLOT_PERIOD_DENOMINATOR;
constexpr uint32_t MOTOR_SLOT_PERIOD_REMAINDER_US =
    1000000UL % MOTOR_SLOT_PERIOD_DENOMINATOR;
constexpr uint32_t MOTOR_CYCLE_PERIOD_BASE_US =
    1000000UL / PICO_B_MOTOR_CYCLE_HZ;
constexpr uint32_t MOTOR_CYCLE_PERIOD_REMAINDER_US =
    1000000UL % PICO_B_MOTOR_CYCLE_HZ;
constexpr uint32_t MOTOR_ONLINE_TIMEOUT_MS = 250;
constexpr uint32_t MOTOR_MODE_SWITCH_GAP_MS = 3;
constexpr uint32_t MOTOR_RATE_WINDOW_MS = 1000;
constexpr uint32_t MOTOR_TEMPERATURE_QUERY_PERIOD_MS = 1000;
constexpr uint32_t MOTOR_TEMPERATURE_PHASE_STEP_MS =
    MOTOR_TEMPERATURE_QUERY_PERIOD_MS / MOTOR_COUNT;
constexpr uint8_t MOTOR_MAX_CONSECUTIVE_ERRORS = 3;
constexpr int16_t MOTOR_TARGET_RPM = 0;
constexpr uint8_t MOTOR_BRAKE_ACTIVE_BYTE = 0xFF;
static_assert(MOTOR_TARGET_RPM == 0, "B3A requires zero RPM");
static_assert(MOTOR_BRAKE_ACTIVE_BYTE == 0xFF,
              "B3A requires active DDSM115 brake");
static_assert(RS485_SPEED_FIRST_ATTEMPT_TOTAL_MS == 15,
              "Speed first-attempt window must remain 15 ms");
static_assert(MOTOR_TEMPERATURE_PHASE_STEP_MS == 250,
              "Temperature queries must remain phase-spaced by 250 ms");
static_assert(PICO_B_MOTOR_CYCLE_HZ == 10 || PICO_B_MOTOR_CYCLE_HZ == 15 ||
                  PICO_B_MOTOR_CYCLE_HZ == 20,
              "PICO_B_MOTOR_CYCLE_HZ must be 10, 15, or 20");

constexpr uint32_t SBUS_BAUD_RATE = 100000;
constexpr uint8_t SBUS_FRAME_SIZE = 25;
constexpr uint8_t SBUS_START_BYTE = 0x0F;
constexpr uint8_t SBUS_END_BYTE = 0x00;
constexpr uint16_t SBUS_MIN = 282;
constexpr uint16_t SBUS_CENTER = 1002;
constexpr uint16_t SBUS_MAX = 1722;
constexpr uint16_t SBUS_SAFE_NEUTRAL = 1002;
constexpr uint16_t SBUS_DEADBAND = 25;
constexpr uint32_t SBUS_TIMEOUT_MS = 100;
constexpr uint32_t SBUS_RATE_WINDOW_MS = 500;
constexpr bool SBUS_RX_INVERTED = true;
constexpr uint16_t OLED_PAGE_PREVIOUS_THRESHOLD = 700;
constexpr uint16_t OLED_PAGE_NEUTRAL_LOW = 900;
constexpr uint16_t OLED_PAGE_NEUTRAL_HIGH = 1100;
constexpr uint16_t OLED_PAGE_NEXT_THRESHOLD = 1300;
constexpr size_t RELAY_COUNT = 2;
constexpr uint8_t RELAY_PINS[RELAY_COUNT] = {RELAY1_PIN, RELAY2_PIN};
constexpr uint8_t RELAY_CHANNEL_INDEX[RELAY_COUNT] = {6, 7};
constexpr uint16_t RELAY_ON_THRESHOLD = 1400;
constexpr uint16_t RELAY_OFF_THRESHOLD = 600;
constexpr uint32_t RELAY_ON_DEBOUNCE_MS = 100;
constexpr uint32_t RELAY_TASK_PERIOD_MS = 20;
constexpr uint8_t RELAY_INHIBIT_NONE = 0;
constexpr uint8_t RELAY_INHIBIT_SBUS = 1;
constexpr uint8_t RELAY_INHIBIT_REARM = 2;
constexpr uint8_t RELAY_INHIBIT_MOTOR = 3;
constexpr uint8_t RELAY_INHIBIT_SYSTEM = 4;
constexpr uint8_t RELAY_INHIBIT_DISABLED = 5;
constexpr int16_t SERVO_MIN_ANGLE_DEG = 0;
constexpr int16_t SERVO_CENTER_ANGLE_DEG = 90;
constexpr int16_t SERVO_MAX_ANGLE_DEG = 180;
constexpr uint16_t SERVO_LEFT_US = 550;
constexpr uint16_t SERVO_CENTER_US = 1570;
constexpr uint16_t SERVO_RIGHT_US = 2450;
constexpr uint16_t SERVO_ABSOLUTE_MIN_US = 500;
constexpr uint16_t SERVO_ABSOLUTE_MAX_US = 2500;
constexpr uint32_t SERVO_UPDATE_PERIOD_MS = 20;
constexpr uint16_t SERVO_MAX_SLEW_DEG_PER_S = 90;
constexpr int16_t SERVO_SLEW_TENTHS_DEG_PER_UPDATE =
    SERVO_MAX_SLEW_DEG_PER_S * 10 * SERVO_UPDATE_PERIOD_MS / 1000U;
constexpr float SERVO_CH4_DEADBAND = 0.02f;
static_assert(RELAY_ACTIVE_LEVEL == LOW && RELAY_INACTIVE_LEVEL == HIGH,
              "Relay outputs must remain active LOW and inactive HIGH");
static_assert(RELAY1_PIN != RELAY2_PIN,
              "Relay GPIO pins must be different");
static_assert(RELAY1_PIN != RS485_BRIDGE_TX_PIN &&
                  RELAY1_PIN != RS485_BRIDGE_RX_PIN &&
                  RELAY1_PIN != SBUS_TX_PIN && RELAY1_PIN != SBUS_RX_PIN &&
                  RELAY1_PIN != CAMERA_SERVO_SIGNAL_PIN &&
                  RELAY1_PIN != OLED_SDA_PIN && RELAY1_PIN != OLED_SCL_PIN &&
                  RELAY1_PIN != BATTERY_ADC_PIN,
              "Relay 1 GP12 conflicts with an active pin");
static_assert(RELAY2_PIN != RS485_BRIDGE_TX_PIN &&
                  RELAY2_PIN != RS485_BRIDGE_RX_PIN &&
                  RELAY2_PIN != SBUS_TX_PIN && RELAY2_PIN != SBUS_RX_PIN &&
                  RELAY2_PIN != CAMERA_SERVO_SIGNAL_PIN &&
                  RELAY2_PIN != OLED_SDA_PIN && RELAY2_PIN != OLED_SCL_PIN &&
                  RELAY2_PIN != BATTERY_ADC_PIN,
              "Relay 2 GP13 conflicts with an active pin");
static_assert(RELAY_CHANNEL_INDEX[0] == 6 && RELAY_CHANNEL_INDEX[1] == 7,
              "Relay channels must remain SBUS CH7 and CH8");
static_assert(RELAY_OFF_THRESHOLD < RELAY_ON_THRESHOLD,
              "Relay OFF threshold must be below ON threshold");
static_assert(CAMERA_SERVO_SIGNAL_PIN != RS485_BRIDGE_TX_PIN &&
                  CAMERA_SERVO_SIGNAL_PIN != RS485_BRIDGE_RX_PIN &&
                  CAMERA_SERVO_SIGNAL_PIN != SBUS_TX_PIN &&
                  CAMERA_SERVO_SIGNAL_PIN != SBUS_RX_PIN &&
                  CAMERA_SERVO_SIGNAL_PIN != OLED_SDA_PIN &&
                  CAMERA_SERVO_SIGNAL_PIN != OLED_SCL_PIN &&
                  CAMERA_SERVO_SIGNAL_PIN != BATTERY_ADC_PIN &&
                  CAMERA_SERVO_SIGNAL_PIN != RELAY1_PIN &&
                  CAMERA_SERVO_SIGNAL_PIN != RELAY2_PIN,
              "Camera servo GP6 conflicts with an active pin");
static_assert(SERVO_MIN_ANGLE_DEG == 0 &&
                  SERVO_CENTER_ANGLE_DEG == 90 &&
                  SERVO_MAX_ANGLE_DEG == 180,
              "RDS3235 angle mapping must remain 0/90/180 degrees");
static_assert(SERVO_ABSOLUTE_MIN_US <= SERVO_LEFT_US);
static_assert(SERVO_LEFT_US == 550);
static_assert(SERVO_CENTER_US == 1570);
static_assert(SERVO_RIGHT_US == 2450);
static_assert(SERVO_RIGHT_US <= SERVO_ABSOLUTE_MAX_US);
static_assert(SERVO_SLEW_TENTHS_DEG_PER_UPDATE == 18,
              "Servo slew must remain 90 deg/s at 20 ms");

constexpr char NODE_NAME[] = "pico_b_control_node";
constexpr char HEARTBEAT_TOPIC[] = "/pico_b/heartbeat";
constexpr char SBUS_TOPIC[] = "/pico_b/sbus/channels";
constexpr char BATTERY_TOPIC[] = "/pico_b/battery/voltage";
constexpr char MOTORS_RPM_TOPIC[] = "/pico_b/motors/rpm";
constexpr char DIAGNOSTICS_TOPIC[] = "/diagnostics";
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
constexpr char CMD_VEL_TOPIC[] = "/cmd_vel";
constexpr size_t CMD_VEL_EXECUTOR_HANDLE_COUNT = 1;
#endif
constexpr char CMD_VEL_DIAG_NAME[] = "pico_b/cmd_vel";
constexpr char WHEEL_COMMAND_TOPIC[] = "/pico_b/wheel_commands";
constexpr char WHEEL_COMMAND_DIAG_NAME[] = "pico_b/wheel_command";
constexpr char WHEEL_FRESHNESS_DIAG_NAME[] = "pico_b/wheel_freshness";
constexpr size_t WHEEL_COMMAND_EXECUTOR_HANDLE_COUNT = 1;
constexpr size_t SBUS_CHANNEL_COUNT = 16;
constexpr size_t DIAG_STATUS_CAPACITY = 1;
constexpr size_t DIAG_VALUE_CAPACITY = 6;
constexpr size_t DIAG_VALUE_TEXT_CAPACITY = 24;

enum class RosState : uint8_t {
  WAIT_AGENT,
  CREATE_ENTITIES,
  RUNNING,
  DESTROY_ENTITIES,
};

enum class DiagnosticPage : uint8_t {
  SBUS,
  BATTERY,
  MICRO_ROS,
  OLED,
  MOTORS,
  MOTOR_1_FL,
  MOTOR_2_RL,
  MOTOR_3_FR,
  MOTOR_4_RR,
  MOTOR_COMM_12,
  MOTOR_COMM_34,
  MOTOR_DETAIL_1,
  MOTOR_DETAIL_2,
  MOTOR_DETAIL_3,
  MOTOR_DETAIL_4,
  MOTOR_LATENCY_12,
  MOTOR_LATENCY_34,
  MOTOR_TEMPERATURE_SCHEDULE,
  MOTOR_SCHEDULER,
  MOTOR_TRANSACTION_GUARD,
  OLED_CONTROL,
  RELAYS,
  CAMERA_SERVO,
  CMD_VEL,
  WHEEL_COMMAND,
  WHEEL_FRESHNESS,
};

enum class OledPage : uint8_t {
  BATTERY,
  SYSTEM,
  MOTORS,
  COUNT,
};

enum class MotorTaskState : uint8_t {
  IDLE,
  SEND_SPEED,
  WAIT_SPEED_RESPONSE,
  WAIT_SPEED_GRACE,
  SEND_SPEED_RETRY,
  WAIT_SPEED_RETRY_RESPONSE,
  SEND_TEMPERATURE,
  WAIT_TEMPERATURE_RESPONSE,
  NEXT_MOTOR,
};

enum class MotorReplyResult : uint8_t {
  VALID,
  TIMEOUT,
  CRC_ERROR,
  LENGTH_ERROR,
  ID_ERROR,
};

enum class SpeedResponsePhase : uint8_t {
  ON_TIME,
  LATE,
  RETRY,
};

struct BatteryStatus {
  float averageRawAdc;
  float adcVoltage;
  float filteredVoltage;
  uint32_t millivolts;
  uint8_t socPercent;
  uint32_t lastUpdateMs;
  uint32_t sampleCount;
  uint32_t adcErrorCount;
  bool valid;
  bool low;
  bool critical;
};

struct SbusStatus {
  uint16_t channels[SBUS_CHANNEL_COUNT];
  bool digitalChannel17;
  bool digitalChannel18;
  bool frameLost;
  bool failsafe;
  bool connected;
  bool lastFrameValid;
  uint32_t validFrameCount;
  uint32_t invalidFrameCount;
  uint32_t frameLostCount;
  uint32_t lastFrameMs;
  float frameRateHz;
};

struct MotorConfig {
  uint8_t id;
  const char *position;
};

struct MotorStatus {
  uint8_t id;
  const char *position;
  bool online;
  int16_t measuredRpm;
  float currentA;
  uint8_t temperatureC;
  uint8_t errorCode;
  uint8_t mode;
  uint32_t validResponseCount;
  uint32_t crcErrorCount;
  uint32_t timeoutCount;
  uint32_t lengthErrorCount;
  uint32_t wrongIdCount;
  uint32_t speedCrcErrorCount;
  uint32_t temperatureCrcErrorCount;
  uint8_t consecutiveFailureCount;
  uint32_t lastValidMs;
  uint32_t sequence;
  uint32_t speedTimeoutCount;
  uint32_t temperatureTimeoutCount;
  uint32_t firstAttemptTimeoutCount;
  uint32_t lateSuccessCount;
  uint32_t lateResponseCount;
  uint32_t retrySuccessCount;
  uint32_t finalTimeoutCount;
  uint32_t responseLatencyLastUs;
  uint32_t responseLatencyWindowMaxUs;
  uint16_t rxLengthLast;
  bool temperatureValid;
};

struct RelayStatus {
  bool requestedOn;
  bool outputOn;
  bool offSeen;
  bool onDebounceActive;
  uint32_t onDebounceStartMs;
};

struct CameraServoStatus {
  bool attached;
  bool inputHealthy;
  int16_t targetRelativeAngleDeg;
  int16_t targetAbsoluteAngleDeg;
  int16_t currentAngleTenthsDeg;
  uint16_t targetPulseUs;
  uint16_t currentPulseUs;
  uint32_t lastUpdateMs;
  uint32_t seenValidFrameCount;
  uint32_t seenInvalidFrameCount;
  bool sbusFrameValid;
};

struct CmdVelStatus {
  float linearX;
  float angularZ;
  uint32_t receiveCount;
  uint32_t lastReceiveMs;
  uint32_t rejectCount;
  bool commandFresh;
  bool commandFinite;
};


enum class WheelRejectReason : uint8_t {
  NONE,
  NULL_MESSAGE,
  PROTOCOL,
  SESSION_ZERO,
  SESSION_CHANGE,
  SEQUENCE,
  TIMESTAMP_ZERO,
  TIMESTAMP_REWIND,
  NONFINITE,
  SOURCE_TOO_OLD,
  SOURCE_IN_FUTURE,
  TIME_UNSYNCED,
  LOCAL_TIMEOUT,
  SOURCE_TIMEOUT,
  EXECUTOR_ERROR,
  LIFECYCLE_RESET,
};

struct WheelCommandStatus {
  double velocityRadS[MOTOR_COUNT];
  uint64_t sourceSessionId;
  uint64_t sampleTimeUnixUs;
  uint32_t sequence;
  uint32_t lastAcceptedRxMs;
  uint32_t receiveCount;
  uint32_t acceptedCount;
  uint32_t rejectCount;
  uint32_t sequenceGapCount;
  uint32_t invalidationCount;
  WheelRejectReason lastRejectReason;
  bool hasCurrentSessionSample;
  bool hasSequence;
  bool localFresh;
  bool sourceTimeMapped;
  bool sourceTimeTrusted;
  bool sourceFresh;
  bool commandFresh;
};
constexpr MotorConfig MOTOR_CONFIGS[MOTOR_COUNT] = {
  {1, "FL"},
  {2, "RL"},
  {3, "FR"},
  {4, "RR"},
};
constexpr int16_t MOTOR_TARGET_RPM_BY_ID[MOTOR_COUNT] = {
  MOTOR_TARGET_RPM,
  MOTOR_TARGET_RPM,
  MOTOR_TARGET_RPM,
  MOTOR_TARGET_RPM,
};
static_assert(MOTOR_COUNT == 4, "Exactly four DDSM115 motors are required");
static_assert(MOTOR_SLOT_PERIOD_US == 25000UL ||
                  MOTOR_SLOT_PERIOD_US == 16666UL ||
                  MOTOR_SLOT_PERIOD_US == 12500UL,
              "Unexpected motor slot period");
static_assert(MOTOR_TARGET_RPM_BY_ID[0] == 0 && MOTOR_TARGET_RPM_BY_ID[1] == 0 &&
                  MOTOR_TARGET_RPM_BY_ID[2] == 0 &&
                  MOTOR_TARGET_RPM_BY_ID[3] == 0,
              "Every motor target must remain 0 RPM");
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
static_assert(MOTOR_TARGET_RPM == 0,
              "B5A: cmd_vel subscriber must NOT enable motor motion; target RPM compile-time 0");
static_assert(MOTOR_BRAKE_ACTIVE_BYTE == 0xFF,
              "B5A: cmd_vel subscriber must NOT enable motor motion; brake byte 0xFF");
#endif
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
static_assert(MOTOR_TARGET_RPM == 0,
              "Wheel RX-only must keep motor target compile-time 0 RPM");
static_assert(MOTOR_BRAKE_ACTIVE_BYTE == 0xFF,
              "Wheel RX-only must keep DDSM115 brake byte 0xFF");
#endif
static_assert(MOTOR_CONFIGS[0].id == 1 && MOTOR_CONFIGS[1].id == 2 &&
                  MOTOR_CONFIGS[2].id == 3 && MOTOR_CONFIGS[3].id == 4,
              "B3A motor ID order must remain ID1 FL, ID2 RL, ID3 FR, ID4 RR");
constexpr uint32_t DDSM115_FRAME_SERIAL_US =
    ((DDSM115_FRAME_SIZE * 10UL * 1000000UL) + RS485_BRIDGE_BAUD_RATE - 1UL) /
    RS485_BRIDGE_BAUD_RATE;
constexpr uint32_t MOTOR_TEMPERATURE_RESPONSE_BUDGET_MIN_US = 3000UL;
constexpr uint32_t MOTOR_TEMPERATURE_GUARD_MARGIN_US = 1000UL;
constexpr uint32_t MOTOR_TEMPERATURE_SLOT_LATE_REJECT_US =
    RS485_FRAME_QUIET_MS * 1000UL;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t heartbeatPublisher;
rcl_publisher_t sbusPublisher;
rcl_publisher_t batteryPublisher;
rcl_publisher_t motorsRpmPublisher;
rcl_publisher_t diagnosticPublisher;
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
rclc_executor_t cmdVelExecutor;
rcl_subscription_t cmdVelSubscriber;
geometry_msgs__msg__Twist cmdVelMsg;
bool cmdVelSubscriberCreated = false;
#endif
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
rclc_executor_t wheelCommandExecutor;
rcl_subscription_t wheelCommandSubscriber;
lessien_interfaces__msg__WheelCommand wheelCommandMsg;
bool wheelCommandExecutorInitialized = false;
bool wheelCommandSubscriberInitialized = false;
static_assert(sizeof(lessien_interfaces__msg__WheelCommand) == 56U,
              "WheelCommand MCU layout must remain fixed at 56 bytes");
static_assert(sizeof(wheelCommandMsg.velocity_rad_s) / sizeof(double) == 4U,
              "WheelCommand must contain exactly FL, RL, FR, RR");
#endif
std_msgs__msg__UInt32 heartbeatMsg;
std_msgs__msg__Int16MultiArray sbusMsg;
std_msgs__msg__Float32 batteryMsg;
std_msgs__msg__Int16MultiArray motorsRpmMsg;
diagnostic_msgs__msg__DiagnosticArray diagnosticMsg;
diagnostic_msgs__msg__DiagnosticStatus diagnosticStatusStorage[DIAG_STATUS_CAPACITY];
diagnostic_msgs__msg__KeyValue diagnosticValueStorage[DIAG_VALUE_CAPACITY];
int16_t sbusMessageData[SBUS_CHANNEL_COUNT];
int16_t motorsRpmMessageData[MOTOR_COUNT];
char diagnosticValueText[DIAG_VALUE_CAPACITY][DIAG_VALUE_TEXT_CAPACITY];
#if PICO_B_ENABLE_OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
#endif
#if PICO_B_ENABLE_CAMERA_SERVO
Servo cameraServoOutput;
#endif

BatteryStatus batteryStatus = {};
SbusStatus sbusStatus = {};
MotorStatus motorStatus[MOTOR_COUNT] = {};
RelayStatus relayStatus[RELAY_COUNT] = {};
CameraServoStatus cameraServoStatus = {};
CmdVelStatus cmdVelStatus = {};
WheelCommandStatus wheelCommandStatus = {};
uint64_t wheelEpochNowUsCached = 0ULL;

RosState rosState = RosState::WAIT_AGENT;
DiagnosticPage diagnosticPage = DiagnosticPage::SBUS;
OledPage oledPage = OledPage::BATTERY;
uint32_t lastAgentPingMs = 0;
uint32_t lastRunningAgentPingMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastSbusPublishMs = 0;
uint32_t lastBatteryMeasurementMs = 0;
uint32_t lastBatteryPublishMs = 0;
uint32_t lastMotorsRpmPublishSequence = 0;
uint32_t lastDiagnosticPublishMs = 0;
uint32_t lastDiagnosticPageMs = 0;
uint32_t lastOledUpdateMs = 0;
uint32_t lastOledPageChangeMs = 0;
uint32_t oledUpdateCount = 0;
uint32_t oledLastUpdateUs = 0;
uint32_t oledWindowMaxUs = 0;
uint32_t oledInitFailureCount = 0;
uint32_t oledRateWindowStartMs = 0;
uint32_t oledRateWindowUpdateCount = 0;
float oledWindowHz = 0.0f;
uint32_t motorDueUs[MOTOR_COUNT] = {};
uint32_t motorDueRemainderUs[MOTOR_COUNT] = {};
uint8_t motorRxFrame[DDSM115_FRAME_SIZE] = {};
uint8_t motorSpeedTxFrame[DDSM115_FRAME_SIZE] = {};
size_t motorRxStored = 0;
size_t motorRxByteCount = 0;
uint32_t motorRequestStartedMs = 0;
uint32_t motorLastRxByteMs = 0;
uint32_t motorRequestStartedUs = 0;
uint32_t motorLastRxByteUs = 0;
uint32_t nextMotorModeSwitchMs = 0;
uint32_t motorTaskLastUs = 0;
uint32_t motorTaskWindowMaxUs = 0;
uint32_t motorRateWindowStartMs = 0;
uint32_t motorRateWindowCycleCount = 0;
float motorTransactionHz = 0.0f;
float motorCycleHz = 0.0f;
uint32_t motorSnapshotSequence = 0;
uint32_t motorSlotMissCount = 0;
uint32_t motorSlotLatenessWindowMaxUs = 0;
uint32_t motorTemperatureDueMs[MOTOR_COUNT] = {};
uint32_t motorTemperatureQueryCount[MOTOR_COUNT] = {};
uint32_t motorTemperatureRateWindowStartMs = 0;
uint32_t motorTemperatureRateWindowCount = 0;
uint32_t motorTemperatureScheduleMissCount = 0;
float motorTemperatureQueryHz = 0.0f;
uint8_t motorTemperaturePendingMask = 0;
bool temperatureQueryThisSlotStarted = false;
uint32_t motorTempStartCount = 0;
uint32_t motorTempDeferredCount = 0;
uint32_t motorTempGuardRejectCount = 0;
uint32_t motorTempStartLatenessWindowMaxUs = 0;
uint8_t speedConsecutiveFailMax = 0;
uint8_t lastOfflineMotorId = 0;
uint32_t activeMotorSlotStartLatenessUs = 0;
uint8_t activeMotorIndex = 0;
uint8_t motorModeSwitchIndex = 0;
bool motorModeSetupComplete = false;
MotorTaskState motorTaskState = MotorTaskState::IDLE;
uint32_t heartbeatSequence = 0;
uint32_t heartbeatPublishError = 0;
uint32_t telemetryPublishError = 0;
uint32_t diagnosticPublishError = 0;
uint32_t reconnectCount = 0;
uint32_t sbusRateWindowStartMs = 0;
uint16_t sbusRateWindowFrameCount = 0;
uint8_t sbusFrameBuffer[SBUS_FRAME_SIZE] = {};
uint8_t sbusFrameIndex = 0;
uint8_t runningAgentPingFailures = 0;
bool transportSerialStarted = false;
bool entitiesCreated = false;
bool batteryFilterInitialized = false;
bool rosSessionError = false;
bool oledAvailable = false;
bool oledInputArmed = false;
bool oledFramePending = false;
uint32_t oledPageChangeCount = 0;
uint32_t oledIgnoredInputCount = 0;
uint32_t relayFailsafeCount = 0;
uint8_t relayInhibitCode = RELAY_INHIBIT_DISABLED;
bool relayOutputSafe = false;
uint32_t lastRelayTaskMs = 0;
uint32_t cameraServoFailsafeCount = 0;

extern "C" bool arduino_transport_open(uxrCustomTransport *transport) {
  (void)transport;
  if (!transportSerialStarted) {
    Serial.begin(115200);
    transportSerialStarted = true;
  }
  return true;
}

extern "C" bool arduino_transport_close(uxrCustomTransport *transport) {
  (void)transport;
  return true;
}

extern "C" size_t arduino_transport_write(uxrCustomTransport *transport,
                                          const uint8_t *buf, size_t len,
                                          uint8_t *errcode) {
  (void)transport;
  (void)errcode;
  return Serial.write(buf, len);
}

extern "C" size_t arduino_transport_read(uxrCustomTransport *transport,
                                         uint8_t *buf, size_t len, int timeout,
                                         uint8_t *errcode) {
  (void)transport;
  (void)errcode;
  Serial.setTimeout(timeout);
  return Serial.readBytes(reinterpret_cast<char *>(buf), len);
}

#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
void invalidateCmdVelSnapshot() {
  cmdVelStatus.linearX = 0.0f;
  cmdVelStatus.angularZ = 0.0f;
  cmdVelStatus.commandFresh = false;
  cmdVelStatus.commandFinite = false;
}

void cmdVelCallback(const void * msgin) {
  const auto *twist = static_cast<const geometry_msgs__msg__Twist *>(msgin);
  if (twist == nullptr) {
    ++cmdVelStatus.rejectCount;
    return;
  }
  const float lx = twist->linear.x;
  const float az = twist->angular.z;
  const bool finite = isfinite(lx) && isfinite(az);
  if (!finite) {
    ++cmdVelStatus.rejectCount;
    cmdVelStatus.commandFinite = false;
    cmdVelStatus.commandFresh = false;
    return;
  }
  const uint32_t nowMs = millis();
  cmdVelStatus.linearX = lx;
  cmdVelStatus.angularZ = az;
  cmdVelStatus.commandFinite = true;
  cmdVelStatus.commandFresh = true;
  cmdVelStatus.lastReceiveMs = nowMs;
  ++cmdVelStatus.receiveCount;
}
#endif

#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
const char *wheelRejectReasonText(WheelRejectReason reason) {
  switch (reason) {
    case WheelRejectReason::NONE: return "NONE";
    case WheelRejectReason::NULL_MESSAGE: return "NULL_MESSAGE";
    case WheelRejectReason::PROTOCOL: return "PROTOCOL";
    case WheelRejectReason::SESSION_ZERO: return "SESSION_ZERO";
    case WheelRejectReason::SESSION_CHANGE: return "SESSION_CHANGE";
    case WheelRejectReason::SEQUENCE: return "SEQUENCE";
    case WheelRejectReason::TIMESTAMP_ZERO: return "TIME_ZERO";
    case WheelRejectReason::TIMESTAMP_REWIND: return "TIME_REWIND";
    case WheelRejectReason::NONFINITE: return "NONFINITE";
    case WheelRejectReason::SOURCE_TOO_OLD: return "SOURCE_OLD";
    case WheelRejectReason::SOURCE_IN_FUTURE: return "SOURCE_FUTURE";
    case WheelRejectReason::TIME_UNSYNCED: return "TIME_UNSYNCED";
    case WheelRejectReason::LOCAL_TIMEOUT: return "LOCAL_TIMEOUT";
    case WheelRejectReason::SOURCE_TIMEOUT: return "SOURCE_TIMEOUT";
    case WheelRejectReason::EXECUTOR_ERROR: return "EXECUTOR_ERROR";
    case WheelRejectReason::LIFECYCLE_RESET: return "LIFECYCLE_RESET";
  }
  return "UNKNOWN";
}

void clearWheelActiveValues() {
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    wheelCommandStatus.velocityRadS[i] = 0.0;
  }
  wheelCommandStatus.localFresh = false;
  wheelCommandStatus.sourceFresh = false;
  wheelCommandStatus.commandFresh = false;
}

void invalidateWheelSnapshot(WheelRejectReason reason, bool resetSession) {
  clearWheelActiveValues();
  ++wheelCommandStatus.invalidationCount;
  wheelCommandStatus.lastRejectReason = reason;
  if (resetSession) {
    wheelCommandStatus.sourceSessionId = 0;
    wheelCommandStatus.sampleTimeUnixUs = 0;
    wheelCommandStatus.sequence = 0;
    wheelCommandStatus.lastAcceptedRxMs = 0;
    wheelCommandStatus.hasCurrentSessionSample = false;
    wheelCommandStatus.hasSequence = false;
    wheelCommandStatus.sourceTimeMapped = false;
    wheelCommandStatus.sourceTimeTrusted = false;
  }
}

void rejectWheelCommand(WheelRejectReason reason) {
  ++wheelCommandStatus.rejectCount;
  invalidateWheelSnapshot(reason, false);
}

constexpr bool wheelSequenceIsNewer(uint32_t sequence, uint32_t previous) {
  const uint32_t delta = sequence - previous;
  return delta != 0U && delta < 0x80000000UL;
}
static_assert(!wheelSequenceIsNewer(7U, 7U), "Duplicate sequence rejected");
static_assert(!wheelSequenceIsNewer(6U, 7U), "Reverse sequence rejected");
static_assert(wheelSequenceIsNewer(0U, 0xFFFFFFFFUL),
              "uint32 sequence wrap accepted");
static_assert(!wheelSequenceIsNewer(0x80000000UL, 0U),
              "Half-range sequence is ambiguous and rejected");

uint64_t wheelEpochNowUs() {
  const int64_t epochNanos = rmw_uros_epoch_nanos();
  return epochNanos > 0 ? static_cast<uint64_t>(epochNanos) / 1000ULL : 0ULL;
}

void wheelCommandCallback(const void *msgin) {
  ++wheelCommandStatus.receiveCount;
  const auto *message =
      static_cast<const lessien_interfaces__msg__WheelCommand *>(msgin);
  if (message == nullptr) {
    rejectWheelCommand(WheelRejectReason::NULL_MESSAGE);
    return;
  }
  if (message->protocol_version != WHEEL_COMMAND_PROTOCOL_VERSION) {
    rejectWheelCommand(WheelRejectReason::PROTOCOL);
    return;
  }
  if (message->source_session_id == 0ULL) {
    rejectWheelCommand(WheelRejectReason::SESSION_ZERO);
    return;
  }
  if (message->sample_time_unix_us == 0ULL) {
    rejectWheelCommand(WheelRejectReason::TIMESTAMP_ZERO);
    return;
  }
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (!isfinite(message->velocity_rad_s[i])) {
      rejectWheelCommand(WheelRejectReason::NONFINITE);
      return;
    }
  }
  if (wheelCommandStatus.hasCurrentSessionSample &&
      message->source_session_id != wheelCommandStatus.sourceSessionId) {
    rejectWheelCommand(WheelRejectReason::SESSION_CHANGE);
    return;
  }
  if (wheelCommandStatus.hasSequence &&
      !wheelSequenceIsNewer(message->sequence, wheelCommandStatus.sequence)) {
    rejectWheelCommand(WheelRejectReason::SEQUENCE);
    return;
  }
  if (wheelCommandStatus.hasCurrentSessionSample &&
      message->sample_time_unix_us < wheelCommandStatus.sampleTimeUnixUs) {
    rejectWheelCommand(WheelRejectReason::TIMESTAMP_REWIND);
    return;
  }

  bool sourceFresh = false;
  if (wheelCommandStatus.sourceTimeTrusted) {
    const uint64_t nowUs = wheelEpochNowUsCached;
    const uint64_t futureToleranceUs =
        static_cast<uint64_t>(WHEEL_COMMAND_SOURCE_FUTURE_TOLERANCE_MS) * 1000ULL;
    if (nowUs == 0ULL ||
        message->sample_time_unix_us > nowUs + futureToleranceUs) {
      rejectWheelCommand(WheelRejectReason::SOURCE_IN_FUTURE);
      return;
    }
    const uint64_t sourceAgeUs =
        nowUs > message->sample_time_unix_us
            ? nowUs - message->sample_time_unix_us
            : 0ULL;
    if (sourceAgeUs >
        static_cast<uint64_t>(WHEEL_COMMAND_SOURCE_ACCEPT_MAX_AGE_MS) * 1000ULL) {
      rejectWheelCommand(WheelRejectReason::SOURCE_TOO_OLD);
      return;
    }
    sourceFresh = true;
  }

  if (wheelCommandStatus.hasSequence) {
    const uint32_t delta = message->sequence - wheelCommandStatus.sequence;
    if (delta > 1U) {
      wheelCommandStatus.sequenceGapCount += delta - 1U;
    }
  }
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    wheelCommandStatus.velocityRadS[i] = message->velocity_rad_s[i];
  }
  wheelCommandStatus.sourceSessionId = message->source_session_id;
  wheelCommandStatus.sampleTimeUnixUs = message->sample_time_unix_us;
  wheelCommandStatus.sequence = message->sequence;
  wheelCommandStatus.lastAcceptedRxMs = millis();
  wheelCommandStatus.hasCurrentSessionSample = true;
  wheelCommandStatus.hasSequence = true;
  wheelCommandStatus.localFresh = true;
  wheelCommandStatus.sourceFresh = sourceFresh;
  wheelCommandStatus.commandFresh =
      wheelCommandStatus.localFresh && wheelCommandStatus.sourceFresh;
  wheelCommandStatus.lastRejectReason =
      wheelCommandStatus.sourceTimeTrusted ? WheelRejectReason::NONE
                                           : WheelRejectReason::TIME_UNSYNCED;
  ++wheelCommandStatus.acceptedCount;
}
#endif

bool periodElapsed(uint32_t nowMs, uint32_t &previousMs, uint32_t periodMs) {
  if (static_cast<uint32_t>(nowMs - previousMs) < periodMs) {
    return false;
  }
  previousMs += periodMs;
  if (static_cast<uint32_t>(nowMs - previousMs) >= periodMs) {
    previousMs = nowMs;
  }
  return true;
}

void setStaticString(rosidl_runtime_c__String &field, char *data,
                     size_t capacity, const char *text) {
  field.data = data;
  field.size = strlcpy(data, text, capacity);
  field.capacity = capacity;
}

void setLiteralString(rosidl_runtime_c__String &field, const char *text) {
  field.data = const_cast<char *>(text);
  field.size = strlen(text);
  field.capacity = field.size + 1;
}

void zeroEntities() {
  support = {};
  node = rcl_get_zero_initialized_node();
  heartbeatPublisher = rcl_get_zero_initialized_publisher();
  sbusPublisher = rcl_get_zero_initialized_publisher();
  batteryPublisher = rcl_get_zero_initialized_publisher();
  motorsRpmPublisher = rcl_get_zero_initialized_publisher();
  diagnosticPublisher = rcl_get_zero_initialized_publisher();
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
  cmdVelSubscriber = rcl_get_zero_initialized_subscription();
  cmdVelExecutor = {};
  cmdVelSubscriberCreated = false;
  cmdVelMsg = {};
#endif
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
  wheelCommandSubscriber = rcl_get_zero_initialized_subscription();
  wheelCommandExecutor = {};
  wheelCommandMsg = {};
  wheelCommandExecutorInitialized = false;
  wheelCommandSubscriberInitialized = false;
#endif
}

void finiPublisher(rcl_publisher_t &publisher) {
  if (publisher.impl != nullptr) {
    const rcl_ret_t ret = rcl_publisher_fini(&publisher, &node);
    (void)ret;
  }
  publisher = rcl_get_zero_initialized_publisher();
}

void destroyEntities() {
  if (support.context.impl != nullptr) {
    rmw_context_t *rmwContext = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmwContext, 0);
  }
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
  if (cmdVelSubscriber.impl != nullptr) {
    const rcl_ret_t retSub = rcl_subscription_fini(&cmdVelSubscriber, &node);
    (void)retSub;
  }
  cmdVelSubscriber = rcl_get_zero_initialized_subscription();
  if (cmdVelSubscriberCreated) {
    const rcl_ret_t retExec = rclc_executor_fini(&cmdVelExecutor);
    (void)retExec;
  }
  cmdVelExecutor = {};
  cmdVelSubscriberCreated = false;
  invalidateCmdVelSnapshot();
#endif
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
  if (wheelCommandExecutorInitialized) {
    const rcl_ret_t retExec = rclc_executor_fini(&wheelCommandExecutor);
    (void)retExec;
  }
  wheelCommandExecutor = {};
  wheelCommandExecutorInitialized = false;
  if (wheelCommandSubscriberInitialized ||
      wheelCommandSubscriber.impl != nullptr) {
    const rcl_ret_t retSub =
        rcl_subscription_fini(&wheelCommandSubscriber, &node);
    (void)retSub;
  }
  wheelCommandSubscriber = rcl_get_zero_initialized_subscription();
  wheelCommandSubscriberInitialized = false;
  invalidateWheelSnapshot(WheelRejectReason::LIFECYCLE_RESET, true);
#endif
  finiPublisher(diagnosticPublisher);
  finiPublisher(motorsRpmPublisher);
  finiPublisher(batteryPublisher);
  finiPublisher(sbusPublisher);
  finiPublisher(heartbeatPublisher);
  if (node.impl != nullptr) {
    const rcl_ret_t ret = rcl_node_fini(&node);
    (void)ret;
  }
  if (support.context.impl != nullptr) {
    (void)rclc_support_fini(&support);
  }
  zeroEntities();
  entitiesCreated = false;
  rosSessionError = false;
  runningAgentPingFailures = 0;
}

rcl_ret_t initBestEffortPublisher(rcl_publisher_t &publisher,
                                  const rosidl_message_type_support_t *type,
                                  const char *topic) {
  rcl_publisher_options_t options = rcl_publisher_get_default_options();
  options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  options.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  options.qos.depth = 1;
  return rcl_publisher_init(&publisher, &node, type, topic, &options);
}

void initSbusMessage() {
  sbusMsg.layout.dim.data = nullptr;
  sbusMsg.layout.dim.size = 0;
  sbusMsg.layout.dim.capacity = 0;
  sbusMsg.layout.data_offset = 0;
  sbusMsg.data.data = sbusMessageData;
  sbusMsg.data.size = SBUS_CHANNEL_COUNT;
  sbusMsg.data.capacity = SBUS_CHANNEL_COUNT;
}

void initMotorsRpmMessage() {
  motorsRpmMsg.layout.dim.data = nullptr;
  motorsRpmMsg.layout.dim.size = 0;
  motorsRpmMsg.layout.dim.capacity = 0;
  motorsRpmMsg.layout.data_offset = 0;
  motorsRpmMsg.data.data = motorsRpmMessageData;
  motorsRpmMsg.data.size = MOTOR_COUNT;
  motorsRpmMsg.data.capacity = MOTOR_COUNT;
}

void initDiagnosticMessage() {
  diagnosticMsg.header.frame_id.data = nullptr;
  diagnosticMsg.header.frame_id.size = 0;
  diagnosticMsg.header.frame_id.capacity = 0;
  diagnosticMsg.status.data = diagnosticStatusStorage;
  diagnosticMsg.status.size = DIAG_STATUS_CAPACITY;
  diagnosticMsg.status.capacity = DIAG_STATUS_CAPACITY;

  diagnostic_msgs__msg__DiagnosticStatus &status = diagnosticStatusStorage[0];
  setLiteralString(status.hardware_id, "pico_b");
  status.values.data = diagnosticValueStorage;
  status.values.size = DIAG_VALUE_CAPACITY;
  status.values.capacity = DIAG_VALUE_CAPACITY;

  for (size_t i = 0; i < DIAG_VALUE_CAPACITY; ++i) {
    diagnosticValueStorage[i].key.data = nullptr;
    diagnosticValueStorage[i].key.size = 0;
    diagnosticValueStorage[i].key.capacity = 0;
    diagnosticValueStorage[i].value.data = diagnosticValueText[i];
    diagnosticValueStorage[i].value.size = 0;
    diagnosticValueStorage[i].value.capacity = DIAG_VALUE_TEXT_CAPACITY;
  }
}

void setDiagnosticValue(size_t index, const char *key, const char *value) {
  if (index >= DIAG_VALUE_CAPACITY) {
    return;
  }
  setLiteralString(diagnosticValueStorage[index].key, key);
  setStaticString(diagnosticValueStorage[index].value, diagnosticValueText[index],
                  DIAG_VALUE_TEXT_CAPACITY, value);
}

void setDiagnosticValueUnsigned(size_t index, const char *key, uint32_t value) {
  if (index >= DIAG_VALUE_CAPACITY) {
    return;
  }
  snprintf(diagnosticValueText[index], DIAG_VALUE_TEXT_CAPACITY, "%lu",
           static_cast<unsigned long>(value));
  setLiteralString(diagnosticValueStorage[index].key, key);
  diagnosticValueStorage[index].value.size = strlen(diagnosticValueText[index]);
}

void setDiagnosticValueUnsigned64(size_t index, const char *key,
                                  uint64_t value) {
  if (index >= DIAG_VALUE_CAPACITY) {
    return;
  }
  snprintf(diagnosticValueText[index], DIAG_VALUE_TEXT_CAPACITY, "%" PRIu64,
           value);
  setLiteralString(diagnosticValueStorage[index].key, key);
  diagnosticValueStorage[index].value.size = strlen(diagnosticValueText[index]);
}

void setDiagnosticValueBool(size_t index, const char *key, bool value) {
  setDiagnosticValue(index, key, value ? "true" : "false");
}

void setDiagnosticValueFloat(size_t index, const char *key, float value,
                             uint8_t decimals) {
  if (index >= DIAG_VALUE_CAPACITY) {
    return;
  }
  const char *format = decimals == 1 ? "%.1f" : "%.2f";
  snprintf(diagnosticValueText[index], DIAG_VALUE_TEXT_CAPACITY, format,
           static_cast<double>(value));
  setLiteralString(diagnosticValueStorage[index].key, key);
  diagnosticValueStorage[index].value.size = strlen(diagnosticValueText[index]);
}

void setDiagnosticValueSigned(size_t index, const char *key, int32_t value) {
  if (index >= DIAG_VALUE_CAPACITY) {
    return;
  }
  snprintf(diagnosticValueText[index], DIAG_VALUE_TEXT_CAPACITY, "%ld",
           static_cast<long>(value));
  setLiteralString(diagnosticValueStorage[index].key, key);
  diagnosticValueStorage[index].value.size = strlen(diagnosticValueText[index]);
}

void setDiagnosticValueHex8(size_t index, const char *key, uint8_t value) {
  if (index >= DIAG_VALUE_CAPACITY) {
    return;
  }
  snprintf(diagnosticValueText[index], DIAG_VALUE_TEXT_CAPACITY, "0x%02X",
           static_cast<unsigned>(value));
  setLiteralString(diagnosticValueStorage[index].key, key);
  diagnosticValueStorage[index].value.size = strlen(diagnosticValueText[index]);
}

void boundedSnprintf(char *buffer, size_t capacity, const char *format,
                     double value) {
  const int written = snprintf(buffer, capacity, format, value);
  if (written < 0) {
    buffer[0] = '\0';
    return;
  }
  if (static_cast<size_t>(written) >= capacity) {
    buffer[capacity - 1] = '\0';
  }
}

const char *batteryStateText() {
  if (!batteryStatus.valid) {
    return "INVALID";
  }
  if (batteryStatus.critical) {
    return "CRITICAL";
  }
  if (batteryStatus.low) {
    return "LOW";
  }
  return "OK";
}

const char *sbusStateText() {
  if (sbusStatus.failsafe) {
    return "FAILSAFE";
  }
  if (!sbusStatus.connected) {
    return "LOST";
  }
  return "OK";
}

const char *oledPageName() {
  switch (oledPage) {
    case OledPage::BATTERY:
      return "BATTERY";
    case OledPage::SYSTEM:
      return "SYSTEM";
    case OledPage::MOTORS:
      return "MOTORS";
    case OledPage::COUNT:
      break;
  }
  return "BATTERY";
}

bool sbusRealHealthyFrame(uint32_t nowMs) {
  return sbusStatus.connected && !sbusStatus.failsafe && sbusStatus.lastFrameValid &&
         static_cast<uint32_t>(nowMs - sbusStatus.lastFrameMs) < SBUS_TIMEOUT_MS;
}

void changeOledPage(int8_t step) {
  uint8_t page = static_cast<uint8_t>(oledPage);
  const uint8_t count = static_cast<uint8_t>(OledPage::COUNT);
  page = static_cast<uint8_t>((page + count + step) % count);
  oledPage = static_cast<OledPage>(page);
  ++oledPageChangeCount;
  oledFramePending = true;
}

void serviceOledManualPaging(uint32_t nowMs) {
#if PICO_B_ENABLE_OLED && PICO_B_ENABLE_MANUAL_OLED_PAGING
  if (!sbusRealHealthyFrame(nowMs)) {
    oledInputArmed = false;
    ++oledIgnoredInputCount;
    return;
  }

  const uint16_t ch3 = sbusStatus.channels[2];
  if (ch3 >= OLED_PAGE_NEUTRAL_LOW && ch3 <= OLED_PAGE_NEUTRAL_HIGH) {
    oledInputArmed = true;
    return;
  }
  if (!oledInputArmed) {
    return;
  }
  if (ch3 >= OLED_PAGE_NEXT_THRESHOLD) {
    changeOledPage(1);
    oledInputArmed = false;
  } else if (ch3 <= OLED_PAGE_PREVIOUS_THRESHOLD) {
    changeOledPage(-1);
    oledInputArmed = false;
  }
#else
  (void)nowMs;
#endif
}

bool motorTelemetryFresh(uint8_t motorIndex, uint32_t nowMs);

float normalizeSbusChannel(uint16_t rawValue) {
  if (rawValue >= SBUS_CENTER - SBUS_DEADBAND &&
      rawValue <= SBUS_CENTER + SBUS_DEADBAND) {
    return 0.0f;
  }

  float normalized;
  if (rawValue < SBUS_CENTER) {
    normalized = (static_cast<float>(rawValue) - SBUS_CENTER) /
                 (SBUS_CENTER - SBUS_MIN);
  } else {
    normalized = (static_cast<float>(rawValue) - SBUS_CENTER) /
                 (SBUS_MAX - SBUS_CENTER);
  }
  return constrain(normalized, -1.0f, 1.0f);
}

bool localMotorAndSystemSafe(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (!motorTelemetryFresh(i, nowMs) || motorStatus[i].errorCode != 0) {
      return false;
    }
  }
#else
  (void)nowMs;
  return false;
#endif
  return batteryStatus.valid && !batteryStatus.critical;
}

void setRelayOutput(uint8_t relayIndex, bool turnOn) {
#if PICO_B_ENABLE_RELAYS
  digitalWrite(RELAY_PINS[relayIndex],
               turnOn ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
  relayStatus[relayIndex].outputOn = turnOn;
#else
  (void)relayIndex;
  (void)turnOn;
#endif
}

void forceAllRelaysOff(uint8_t inhibitCode) {
  relayInhibitCode = inhibitCode;
  relayOutputSafe = false;
  for (uint8_t relayIndex = 0; relayIndex < RELAY_COUNT; ++relayIndex) {
    RelayStatus &status = relayStatus[relayIndex];
    status.requestedOn = false;
    status.offSeen = false;
    status.onDebounceActive = false;
    setRelayOutput(relayIndex, false);
  }
}

uint8_t relaySafetyInhibit(uint32_t nowMs) {
#if PICO_B_ENABLE_RELAYS
  if (!sbusRealHealthyFrame(nowMs) || sbusStatus.frameLost) {
    return RELAY_INHIBIT_SBUS;
  }
  if (!localMotorAndSystemSafe(nowMs)) {
    return batteryStatus.valid && !batteryStatus.critical
               ? RELAY_INHIBIT_MOTOR
               : RELAY_INHIBIT_SYSTEM;
  }
  return RELAY_INHIBIT_NONE;
#else
  (void)nowMs;
  return RELAY_INHIBIT_DISABLED;
#endif
}

void updateRelayFromChannel(uint8_t relayIndex, uint16_t raw, uint32_t nowMs) {
  RelayStatus &status = relayStatus[relayIndex];
  if (raw <= RELAY_OFF_THRESHOLD) {
    status.requestedOn = false;
    status.offSeen = true;
    status.onDebounceActive = false;
    setRelayOutput(relayIndex, false);
    return;
  }
  if (raw < RELAY_ON_THRESHOLD) {
    status.requestedOn = false;
    status.onDebounceActive = false;
    setRelayOutput(relayIndex, false);
    return;
  }

  status.requestedOn = true;
  if (!status.offSeen) {
    status.onDebounceActive = false;
    setRelayOutput(relayIndex, false);
    return;
  }
  if (!status.onDebounceActive) {
    status.onDebounceActive = true;
    status.onDebounceStartMs = nowMs;
    return;
  }
  if (static_cast<uint32_t>(nowMs - status.onDebounceStartMs) >=
      RELAY_ON_DEBOUNCE_MS) {
    status.onDebounceActive = false;
    setRelayOutput(relayIndex, true);
  }
}

uint8_t relayNormalInhibit() {
  for (uint8_t relayIndex = 0; relayIndex < RELAY_COUNT; ++relayIndex) {
    const RelayStatus &status = relayStatus[relayIndex];
    if (status.requestedOn && !status.offSeen) {
      return RELAY_INHIBIT_REARM;
    }
  }
  return RELAY_INHIBIT_NONE;
}

void initializeRelayControl(uint32_t nowMs) {
  for (uint8_t relayIndex = 0; relayIndex < RELAY_COUNT; ++relayIndex) {
    RelayStatus &status = relayStatus[relayIndex];
    status.requestedOn = false;
    status.outputOn = false;
    status.offSeen = false;
    status.onDebounceActive = false;
    status.onDebounceStartMs = nowMs;
    setRelayOutput(relayIndex, false);
  }
  relayFailsafeCount = 0;
  relayInhibitCode =
      PICO_B_ENABLE_RELAYS ? RELAY_INHIBIT_SBUS : RELAY_INHIBIT_DISABLED;
  relayOutputSafe = false;
  lastRelayTaskMs = nowMs;
}

void relayTask(uint32_t nowMs) {
  const uint8_t safetyInhibit = relaySafetyInhibit(nowMs);
  if (safetyInhibit != RELAY_INHIBIT_NONE) {
    if (relayOutputSafe || relayInhibitCode == RELAY_INHIBIT_NONE) {
      ++relayFailsafeCount;
    }
    forceAllRelaysOff(safetyInhibit);
    return;
  }

  relayOutputSafe = true;
  relayInhibitCode = RELAY_INHIBIT_NONE;
  for (uint8_t relayIndex = 0; relayIndex < RELAY_COUNT; ++relayIndex) {
    const uint16_t raw = sbusStatus.channels[RELAY_CHANNEL_INDEX[relayIndex]];
    if (raw < RELAY_ON_THRESHOLD) {
      RelayStatus &status = relayStatus[relayIndex];
      status.requestedOn = false;
      status.onDebounceActive = false;
      if (raw <= RELAY_OFF_THRESHOLD) {
        status.offSeen = true;
      }
      setRelayOutput(relayIndex, false);
    }
  }
  if (static_cast<uint32_t>(nowMs - lastRelayTaskMs) < RELAY_TASK_PERIOD_MS) {
    return;
  }
  lastRelayTaskMs = nowMs;

  for (uint8_t relayIndex = 0; relayIndex < RELAY_COUNT; ++relayIndex) {
    updateRelayFromChannel(
        relayIndex, sbusStatus.channels[RELAY_CHANNEL_INDEX[relayIndex]], nowMs);
  }
  relayInhibitCode = relayNormalInhibit();
}

int16_t cameraServoTargetRelativeAngleFromCh4() {
  const float ch4Normalized = normalizeSbusChannel(sbusStatus.channels[3]);
  if (fabs(ch4Normalized) <= SERVO_CH4_DEADBAND) {
    return 0;
  }
  const float relativeAngle = -ch4Normalized * 90.0f;
  const int16_t roundedAngle =
      static_cast<int16_t>(relativeAngle >= 0.0f ? relativeAngle + 0.5f
                                                 : relativeAngle - 0.5f);
  return constrain(roundedAngle, -90, 90);
}

uint16_t cameraServoAngleToPulseUs(float angleDeg) {
  if (!isfinite(angleDeg)) {
    angleDeg = SERVO_CENTER_ANGLE_DEG;
  }
  const float limitedAngleDeg =
      constrain(angleDeg, static_cast<float>(SERVO_MIN_ANGLE_DEG),
                static_cast<float>(SERVO_MAX_ANGLE_DEG));
  int32_t pulseUs;
  if (limitedAngleDeg <= SERVO_CENTER_ANGLE_DEG) {
    const float fraction =
        (limitedAngleDeg - SERVO_MIN_ANGLE_DEG) /
        (SERVO_CENTER_ANGLE_DEG - SERVO_MIN_ANGLE_DEG);
    pulseUs = static_cast<int32_t>(
        SERVO_LEFT_US + fraction * (SERVO_CENTER_US - SERVO_LEFT_US) + 0.5f);
  } else {
    const float fraction =
        (limitedAngleDeg - SERVO_CENTER_ANGLE_DEG) /
        (SERVO_MAX_ANGLE_DEG - SERVO_CENTER_ANGLE_DEG);
    pulseUs = static_cast<int32_t>(
        SERVO_CENTER_US + fraction * (SERVO_RIGHT_US - SERVO_CENTER_US) +
        0.5f);
  }
  return static_cast<uint16_t>(constrain(
      pulseUs, static_cast<int32_t>(SERVO_LEFT_US),
      static_cast<int32_t>(SERVO_RIGHT_US)));
}

uint16_t cameraServoPulseFromAngleTenths(int16_t angleTenthsDeg) {
  return cameraServoAngleToPulseUs(
      static_cast<float>(angleTenthsDeg) / 10.0f);
}

void initializeCameraServo(uint32_t nowMs) {
  cameraServoStatus.attached = false;
  cameraServoStatus.inputHealthy = false;
  cameraServoStatus.targetRelativeAngleDeg = 0;
  cameraServoStatus.targetAbsoluteAngleDeg = SERVO_CENTER_ANGLE_DEG;
  cameraServoStatus.currentAngleTenthsDeg = SERVO_CENTER_ANGLE_DEG * 10;
  cameraServoStatus.targetPulseUs = SERVO_CENTER_US;
  cameraServoStatus.currentPulseUs = SERVO_CENTER_US;
  cameraServoStatus.lastUpdateMs = nowMs;
  cameraServoStatus.seenValidFrameCount = sbusStatus.validFrameCount;
  cameraServoStatus.seenInvalidFrameCount = sbusStatus.invalidFrameCount;
  cameraServoStatus.sbusFrameValid = false;
  cameraServoFailsafeCount = 0;
#if PICO_B_ENABLE_CAMERA_SERVO
  cameraServoStatus.attached =
      cameraServoOutput.attach(CAMERA_SERVO_SIGNAL_PIN,
                               SERVO_ABSOLUTE_MIN_US,
                               SERVO_ABSOLUTE_MAX_US,
                               SERVO_CENTER_US) >= 0;
  if (cameraServoStatus.attached) {
    cameraServoOutput.writeMicroseconds(SERVO_CENTER_US);
  }
#endif
}

void cameraServoTask(uint32_t nowMs) {
#if PICO_B_ENABLE_CAMERA_SERVO
  if (!cameraServoStatus.attached) {
    return;
  }

  const bool newInvalidFrame =
      sbusStatus.invalidFrameCount != cameraServoStatus.seenInvalidFrameCount;
  const bool newValidFrame =
      sbusStatus.validFrameCount != cameraServoStatus.seenValidFrameCount;
  cameraServoStatus.seenInvalidFrameCount = sbusStatus.invalidFrameCount;
  cameraServoStatus.seenValidFrameCount = sbusStatus.validFrameCount;
  if (newInvalidFrame) {
    cameraServoStatus.sbusFrameValid = false;
  } else if (newValidFrame) {
    cameraServoStatus.sbusFrameValid = true;
  }

  const bool inputHealthy =
      cameraServoStatus.sbusFrameValid && sbusRealHealthyFrame(nowMs) &&
      !sbusStatus.frameLost;
  if (cameraServoStatus.inputHealthy && !inputHealthy) {
    ++cameraServoFailsafeCount;
  }
  cameraServoStatus.inputHealthy = inputHealthy;
  cameraServoStatus.targetRelativeAngleDeg =
      inputHealthy ? cameraServoTargetRelativeAngleFromCh4() : 0;
  cameraServoStatus.targetAbsoluteAngleDeg = constrain(
      SERVO_CENTER_ANGLE_DEG + cameraServoStatus.targetRelativeAngleDeg,
      SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);
  cameraServoStatus.targetPulseUs = cameraServoPulseFromAngleTenths(
      cameraServoStatus.targetAbsoluteAngleDeg * 10);

  if (static_cast<uint32_t>(nowMs - cameraServoStatus.lastUpdateMs) <
      SERVO_UPDATE_PERIOD_MS) {
    return;
  }
  cameraServoStatus.lastUpdateMs = nowMs;

  const int16_t targetAngleTenthsDeg =
      cameraServoStatus.targetAbsoluteAngleDeg * 10;
  if (cameraServoStatus.currentAngleTenthsDeg < targetAngleTenthsDeg) {
    const int16_t nextAngle = cameraServoStatus.currentAngleTenthsDeg +
                              SERVO_SLEW_TENTHS_DEG_PER_UPDATE;
    cameraServoStatus.currentAngleTenthsDeg =
        nextAngle < targetAngleTenthsDeg ? nextAngle : targetAngleTenthsDeg;
  } else if (cameraServoStatus.currentAngleTenthsDeg > targetAngleTenthsDeg) {
    const int16_t nextAngle = cameraServoStatus.currentAngleTenthsDeg -
                              SERVO_SLEW_TENTHS_DEG_PER_UPDATE;
    cameraServoStatus.currentAngleTenthsDeg =
        nextAngle > targetAngleTenthsDeg ? nextAngle : targetAngleTenthsDeg;
  }

  cameraServoStatus.currentPulseUs = cameraServoPulseFromAngleTenths(
      cameraServoStatus.currentAngleTenthsDeg);
  cameraServoStatus.currentPulseUs = constrain(
      cameraServoStatus.currentPulseUs, SERVO_LEFT_US, SERVO_RIGHT_US);
  cameraServoOutput.writeMicroseconds(cameraServoStatus.currentPulseUs);
#else
  (void)nowMs;
#endif
}

uint8_t ddsm115Crc8MaximUpdate(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 0x01U) ? ((crc >> 1) ^ 0x8CU) : (crc >> 1);
  }
  return crc;
}

uint8_t ddsm115Crc8Maxim(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc = ddsm115Crc8MaximUpdate(crc, data[i]);
  }
  return crc;
}

bool motorTimeReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

uint32_t motorSlotOffsetUs(uint8_t motorIndex) {
  return (static_cast<uint32_t>(motorIndex) * 1000000UL) /
         MOTOR_SLOT_PERIOD_DENOMINATOR;
}

void advanceMotorCycleDeadline(uint8_t motorIndex) {
  motorDueUs[motorIndex] += MOTOR_CYCLE_PERIOD_BASE_US;
  motorDueRemainderUs[motorIndex] += MOTOR_CYCLE_PERIOD_REMAINDER_US;
  if (motorDueRemainderUs[motorIndex] >= PICO_B_MOTOR_CYCLE_HZ) {
    ++motorDueUs[motorIndex];
    motorDueRemainderUs[motorIndex] -= PICO_B_MOTOR_CYCLE_HZ;
  }
}

void updateMotorTemperatureRate(uint32_t nowMs);
void advanceMotorTemperatureDeadline(uint8_t motorIndex, uint32_t nowMs);
void skipMotorTemperatureDeadline(uint8_t motorIndex, uint32_t nowMs);
bool motorTemperatureWindowAvailable();
void finishSpeedTransaction(uint32_t nowMs);

int16_t readDdsm115SignedBigEndian(const uint8_t *data) {
  const uint16_t raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  return static_cast<int16_t>(raw);
}

bool motorTelemetryFresh(uint8_t motorIndex, uint32_t nowMs) {
  const MotorStatus &status = motorStatus[motorIndex];
  return status.online &&
         static_cast<uint32_t>(nowMs - status.lastValidMs) <=
             MOTOR_ONLINE_TIMEOUT_MS;
}

void setMotorOnline(uint8_t motorIndex, bool online) {
  motorStatus[motorIndex].online = online;
}

void updateMotorOnlineStates(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    MotorStatus &status = motorStatus[i];
    if (status.online &&
        static_cast<uint32_t>(nowMs - status.lastValidMs) >
            MOTOR_ONLINE_TIMEOUT_MS) {
      lastOfflineMotorId = status.id;
      setMotorOnline(i, false);
    }
  }
#else
  (void)nowMs;
#endif
}

void discardPendingMotorRx() {
#if PICO_B_ENABLE_MOTORS
  for (uint8_t i = 0; i < 32 && Serial1.available() > 0; ++i) {
    (void)Serial1.read();
  }
#endif
}

void resetMotorRx(uint32_t nowMs) {
  motorRxStored = 0;
  motorRxByteCount = 0;
  motorRequestStartedMs = nowMs;
  motorLastRxByteMs = nowMs;
  motorRequestStartedUs = micros();
  motorLastRxByteUs = motorRequestStartedUs;
}

void sendMotorVelocityModeSwitch(uint8_t motorIndex) {
#if PICO_B_ENABLE_MOTORS
  const uint8_t frame[DDSM115_FRAME_SIZE] = {
      MOTOR_CONFIGS[motorIndex].id, 0xA0, 0x00, 0x00, 0x00,
      0x00,                         0x00, 0x00, 0x00, 0x02,
  };
  Serial1.write(frame, sizeof(frame));
#else
  (void)motorIndex;
#endif
}

void sendMotorSpeedCommand(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  discardPendingMotorRx();
  memset(motorSpeedTxFrame, 0, sizeof(motorSpeedTxFrame));
  const int16_t targetRpm = MOTOR_TARGET_RPM_BY_ID[activeMotorIndex];
  static_assert(MOTOR_TARGET_RPM == 0, "B3A requires zero RPM");

  motorSpeedTxFrame[0] = MOTOR_CONFIGS[activeMotorIndex].id;
  motorSpeedTxFrame[1] = 0x64;
  motorSpeedTxFrame[2] = static_cast<uint16_t>(targetRpm) >> 8;
  motorSpeedTxFrame[3] = static_cast<uint16_t>(targetRpm) & 0xFF;
  motorSpeedTxFrame[6] = 0x00;
  motorSpeedTxFrame[7] = MOTOR_BRAKE_ACTIVE_BYTE;
  motorSpeedTxFrame[9] =
      ddsm115Crc8Maxim(motorSpeedTxFrame, DDSM115_FRAME_SIZE - 1);

  resetMotorRx(nowMs);
  Serial1.write(motorSpeedTxFrame, sizeof(motorSpeedTxFrame));
#else
  (void)nowMs;
#endif
}

void retryMotorSpeedCommand(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  discardPendingMotorRx();
  resetMotorRx(nowMs);
  Serial1.write(motorSpeedTxFrame, sizeof(motorSpeedTxFrame));
#else
  (void)nowMs;
#endif
}

void sendMotorTemperatureQuery(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  discardPendingMotorRx();
  uint8_t frame[DDSM115_FRAME_SIZE] = {};
  frame[0] = MOTOR_CONFIGS[activeMotorIndex].id;
  frame[1] = 0x74;
  frame[9] = ddsm115Crc8Maxim(frame, DDSM115_FRAME_SIZE - 1);

  resetMotorRx(nowMs);
  Serial1.write(frame, sizeof(frame));
  ++motorTemperatureQueryCount[activeMotorIndex];
  ++motorTemperatureRateWindowCount;
  ++motorTempStartCount;
  if (activeMotorSlotStartLatenessUs > motorTempStartLatenessWindowMaxUs) {
    motorTempStartLatenessWindowMaxUs = activeMotorSlotStartLatenessUs;
  }
  motorTemperaturePendingMask &= static_cast<uint8_t>(~(1U << activeMotorIndex));
  temperatureQueryThisSlotStarted = true;
  advanceMotorTemperatureDeadline(activeMotorIndex, nowMs);
  updateMotorTemperatureRate(nowMs);
#else
  (void)nowMs;
#endif
}

void storeValidMotorFeedback(bool temperatureResponse, uint32_t nowMs) {
  MotorStatus &status = motorStatus[activeMotorIndex];
  const int16_t currentRaw = readDdsm115SignedBigEndian(&motorRxFrame[2]);
  status.mode = motorRxFrame[1];
  status.currentA = currentRaw * (8.0f / 32767.0f);
  status.measuredRpm = readDdsm115SignedBigEndian(&motorRxFrame[4]);
  status.errorCode = motorRxFrame[8];
  if (temperatureResponse) {
    status.temperatureC = motorRxFrame[6];
    status.temperatureValid = true;
  }
  ++status.validResponseCount;
  ++status.sequence;
  if (!temperatureResponse) {
    status.lastValidMs = nowMs;
    status.consecutiveFailureCount = 0;
    setMotorOnline(activeMotorIndex, true);
  }
}

void recordMotorCommunicationError(MotorReplyResult result,
                                   bool temperatureResponse) {
  MotorStatus &status = motorStatus[activeMotorIndex];
  status.rxLengthLast = static_cast<uint16_t>(
      motorRxByteCount > 0xFFFFU ? 0xFFFFU : motorRxByteCount);
  switch (result) {
    case MotorReplyResult::TIMEOUT:
      ++status.timeoutCount;
      if (temperatureResponse) {
        ++status.temperatureTimeoutCount;
      } else {
        ++status.speedTimeoutCount;
      }
      break;
    case MotorReplyResult::CRC_ERROR:
      ++status.crcErrorCount;
      if (temperatureResponse) {
        ++status.temperatureCrcErrorCount;
      } else {
        ++status.speedCrcErrorCount;
      }
      break;
    case MotorReplyResult::LENGTH_ERROR:
      ++status.lengthErrorCount;
      break;
    case MotorReplyResult::ID_ERROR:
      ++status.wrongIdCount;
      break;
    case MotorReplyResult::VALID:
      return;
  }

  if (temperatureResponse) {
    return;
  }

  if (status.consecutiveFailureCount < 0xFF) {
    ++status.consecutiveFailureCount;
  }
  if (status.consecutiveFailureCount > speedConsecutiveFailMax) {
    speedConsecutiveFailMax = status.consecutiveFailureCount;
  }
  if (status.consecutiveFailureCount >= MOTOR_MAX_CONSECUTIVE_ERRORS) {
    lastOfflineMotorId = status.id;
    setMotorOnline(activeMotorIndex, false);
  }
}

MotorReplyResult validateMotorResponse(bool timedOut) {
  if (timedOut) {
    return MotorReplyResult::TIMEOUT;
  }
  if (motorRxByteCount != DDSM115_FRAME_SIZE ||
      motorRxStored != DDSM115_FRAME_SIZE) {
    return MotorReplyResult::LENGTH_ERROR;
  }
  if (ddsm115Crc8Maxim(motorRxFrame, DDSM115_FRAME_SIZE - 1) !=
      motorRxFrame[DDSM115_FRAME_SIZE - 1]) {
    return MotorReplyResult::CRC_ERROR;
  }
  if (motorRxFrame[0] != MOTOR_CONFIGS[activeMotorIndex].id) {
    return MotorReplyResult::ID_ERROR;
  }
  return MotorReplyResult::VALID;
}

bool collectMotorResponse(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  uint8_t readBudget = 32;
  while (readBudget-- > 0 && Serial1.available() > 0) {
    const int value = Serial1.read();
    if (value < 0) {
      break;
    }
    if (motorRxStored < DDSM115_FRAME_SIZE) {
      motorRxFrame[motorRxStored++] = static_cast<uint8_t>(value);
    }
    ++motorRxByteCount;
    motorLastRxByteMs = nowMs;
    motorLastRxByteUs = micros();
  }

  return motorRxByteCount >= DDSM115_FRAME_SIZE &&
         static_cast<uint32_t>(nowMs - motorLastRxByteMs) >=
             RS485_FRAME_QUIET_MS;
#else
  (void)nowMs;
  return false;
#endif
}

void finishMotorResponse(bool temperatureResponse, bool timedOut,
                         uint32_t nowMs) {
  MotorStatus &status = motorStatus[activeMotorIndex];
  status.rxLengthLast = static_cast<uint16_t>(
      motorRxByteCount > 0xFFFFU ? 0xFFFFU : motorRxByteCount);
  const MotorReplyResult result = validateMotorResponse(timedOut);
  if (result != MotorReplyResult::TIMEOUT) {
    const uint32_t latencyUs = motorLastRxByteUs - motorRequestStartedUs;
    status.responseLatencyLastUs = latencyUs;
    if (latencyUs > status.responseLatencyWindowMaxUs) {
      status.responseLatencyWindowMaxUs = latencyUs;
    }
  }
  if (result == MotorReplyResult::VALID) {
    storeValidMotorFeedback(temperatureResponse, nowMs);
  } else {
    recordMotorCommunicationError(result, temperatureResponse);
  }
  motorTaskState = MotorTaskState::NEXT_MOTOR;
}

void finishSpeedResponse(SpeedResponsePhase phase, uint32_t nowMs) {
  MotorStatus &status = motorStatus[activeMotorIndex];
  status.rxLengthLast = static_cast<uint16_t>(
      motorRxByteCount > 0xFFFFU ? 0xFFFFU : motorRxByteCount);
  const MotorReplyResult result = validateMotorResponse(false);
  const uint32_t latencyUs = motorLastRxByteUs - motorRequestStartedUs;
  status.responseLatencyLastUs = latencyUs;
  if (latencyUs > status.responseLatencyWindowMaxUs) {
    status.responseLatencyWindowMaxUs = latencyUs;
  }
  if (result == MotorReplyResult::VALID) {
    if (phase == SpeedResponsePhase::LATE) {
      ++status.lateSuccessCount;
      ++status.lateResponseCount;
    } else if (phase == SpeedResponsePhase::RETRY) {
      ++status.retrySuccessCount;
    }
    storeValidMotorFeedback(false, nowMs);
  } else {
    recordMotorCommunicationError(result, false);
  }
  finishSpeedTransaction(nowMs);
}

void serviceTemperatureResponse(uint32_t nowMs) {
  if (collectMotorResponse(nowMs)) {
    finishMotorResponse(true, false, nowMs);
  } else if (static_cast<uint32_t>(nowMs - motorRequestStartedMs) >=
             RS485_RESPONSE_TIMEOUT_MS) {
    finishMotorResponse(true, true, nowMs);
  }
}

void updateMotorRate(uint32_t nowMs) {
  ++motorRateWindowCycleCount;
  if (motorRateWindowStartMs == 0) {
    motorRateWindowStartMs = nowMs;
    return;
  }

  const uint32_t elapsedMs = nowMs - motorRateWindowStartMs;
  if (elapsedMs >= MOTOR_RATE_WINDOW_MS) {
    motorTransactionHz = motorRateWindowCycleCount * 1000.0f / elapsedMs;
    motorCycleHz = motorTransactionHz / MOTOR_COUNT;
    motorRateWindowStartMs = nowMs;
    motorRateWindowCycleCount = 0;
    motorTaskWindowMaxUs = motorTaskLastUs;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      motorStatus[i].responseLatencyWindowMaxUs =
          motorStatus[i].responseLatencyLastUs;
    }
  }
}

bool motorTemperatureDue(uint8_t motorIndex, uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  return motorTimeReached(nowMs, motorTemperatureDueMs[motorIndex]);
#else
  (void)motorIndex;
  (void)nowMs;
  return false;
#endif
}

void updateMotorTemperatureRate(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  if (motorTemperatureRateWindowStartMs == 0) {
    motorTemperatureRateWindowStartMs = nowMs;
    return;
  }

  const uint32_t elapsedMs = nowMs - motorTemperatureRateWindowStartMs;
  if (elapsedMs >= MOTOR_RATE_WINDOW_MS) {
    motorTemperatureQueryHz =
        motorTemperatureRateWindowCount * 1000.0f / elapsedMs;
    motorTemperatureRateWindowStartMs = nowMs;
    motorTemperatureRateWindowCount = 0;
  }
#else
  (void)nowMs;
#endif
}

void advanceMotorTemperatureDeadline(uint8_t motorIndex, uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  motorTemperatureDueMs[motorIndex] += MOTOR_TEMPERATURE_QUERY_PERIOD_MS;
  if (motorTimeReached(nowMs, motorTemperatureDueMs[motorIndex])) {
    motorTemperatureDueMs[motorIndex] = nowMs + MOTOR_TEMPERATURE_QUERY_PERIOD_MS;
  }
#else
  (void)motorIndex;
  (void)nowMs;
#endif
}

void skipMotorTemperatureDeadline(uint8_t motorIndex, uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  const uint8_t pendingBit = static_cast<uint8_t>(1U << motorIndex);
  if ((motorTemperaturePendingMask & pendingBit) == 0U) {
    ++motorTempDeferredCount;
    ++motorTemperatureScheduleMissCount;
  }
  motorTemperaturePendingMask |= pendingBit;
  (void)nowMs;
#else
  (void)motorIndex;
  (void)nowMs;
#endif
}

bool motorTemperatureWindowAvailable() {
#if PICO_B_ENABLE_MOTORS
  const uint32_t nowUs = micros();
  if (activeMotorSlotStartLatenessUs > MOTOR_TEMPERATURE_SLOT_LATE_REJECT_US) {
    ++motorTempGuardRejectCount;
    return false;
  }
  const uint8_t nextMotorIndex = (activeMotorIndex + 1) % MOTOR_COUNT;
  const uint32_t observedResponseUs =
      motorStatus[activeMotorIndex].responseLatencyWindowMaxUs > 3000UL
          ? motorStatus[activeMotorIndex].responseLatencyWindowMaxUs
          : 3000UL;
  const uint32_t responseBudgetUs =
      observedResponseUs > MOTOR_TEMPERATURE_RESPONSE_BUDGET_MIN_US
          ? observedResponseUs
          : MOTOR_TEMPERATURE_RESPONSE_BUDGET_MIN_US;
  const uint32_t latestSafeUs =
      nowUs + DDSM115_FRAME_SERIAL_US + responseBudgetUs +
      RS485_FRAME_QUIET_MS * 1000UL + MOTOR_TEMPERATURE_GUARD_MARGIN_US;
  if (motorTimeReached(latestSafeUs, motorDueUs[nextMotorIndex])) {
    ++motorTempGuardRejectCount;
    return false;
  }
  return true;
#else
  return false;
#endif
}

void finishSpeedTransaction(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  const bool hasPendingOrDue =
      ((motorTemperaturePendingMask & static_cast<uint8_t>(1U << activeMotorIndex)) != 0U) ||
      motorTemperatureDue(activeMotorIndex, nowMs);
  if (hasPendingOrDue) {
    if (!temperatureQueryThisSlotStarted && motorTemperatureWindowAvailable()) {
      motorTemperaturePendingMask &= static_cast<uint8_t>(~(1U << activeMotorIndex));
      temperatureQueryThisSlotStarted = true;
      motorTaskState = MotorTaskState::SEND_TEMPERATURE;
      return;
    }
    skipMotorTemperatureDeadline(activeMotorIndex, nowMs);
  }
  motorTaskState = MotorTaskState::NEXT_MOTOR;
#else
  (void)nowMs;
#endif
}

void initializeMotorTask(uint32_t nowMs) {
  const uint32_t nowUs = micros();
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    motorStatus[i].id = MOTOR_CONFIGS[i].id;
    motorStatus[i].position = MOTOR_CONFIGS[i].position;
    motorStatus[i].online = false;
    motorStatus[i].measuredRpm = 0;
    motorStatus[i].currentA = 0.0f;
    motorStatus[i].temperatureC = 0;
    motorStatus[i].errorCode = 0;
    motorStatus[i].mode = 0;
    motorStatus[i].lastValidMs = nowMs;
    motorDueUs[i] = nowUs + 10000UL + motorSlotOffsetUs(i);
    motorDueRemainderUs[i] = 0;
    motorTemperatureDueMs[i] = nowMs + i * MOTOR_TEMPERATURE_PHASE_STEP_MS;
    motorTemperatureQueryCount[i] = 0;
  }
  activeMotorIndex = 0;
  motorModeSwitchIndex = 0;
  motorModeSetupComplete = false;
  nextMotorModeSwitchMs = nowMs;
  motorTaskState = MotorTaskState::IDLE;
  motorTaskLastUs = 0;
  motorTaskWindowMaxUs = 0;
  motorRateWindowStartMs = nowMs;
  motorRateWindowCycleCount = 0;
  motorTransactionHz = 0.0f;
  motorCycleHz = 0.0f;
  motorSnapshotSequence = 0;
  motorSlotMissCount = 0;
  motorSlotLatenessWindowMaxUs = 0;
  motorTemperatureRateWindowStartMs = nowMs;
  motorTemperatureRateWindowCount = 0;
  motorTemperatureScheduleMissCount = 0;
  motorTemperatureQueryHz = 0.0f;
  motorTemperaturePendingMask = 0;
  temperatureQueryThisSlotStarted = false;
  motorTempStartCount = 0;
  motorTempDeferredCount = 0;
  motorTempGuardRejectCount = 0;
  motorTempStartLatenessWindowMaxUs = 0;
  speedConsecutiveFailMax = 0;
  lastOfflineMotorId = 0;
  activeMotorSlotStartLatenessUs = 0;
}

void serviceMotorStateMachine(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  const uint32_t nowUs = micros();
  updateMotorOnlineStates(nowMs);

  switch (motorTaskState) {
    case MotorTaskState::IDLE:
      if (!motorModeSetupComplete) {
        if (!motorTimeReached(nowMs, nextMotorModeSwitchMs)) {
          return;
        }
        discardPendingMotorRx();
        sendMotorVelocityModeSwitch(motorModeSwitchIndex);
        ++motorModeSwitchIndex;
        nextMotorModeSwitchMs = nowMs + MOTOR_MODE_SWITCH_GAP_MS;
        if (motorModeSwitchIndex == MOTOR_COUNT) {
          motorModeSetupComplete = true;
          const uint32_t firstSlotUs = nowUs + 10000UL;
          for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
            motorDueUs[i] = firstSlotUs + motorSlotOffsetUs(i);
            motorDueRemainderUs[i] = 0;
          }
          activeMotorIndex = 0;
        }
        return;
      }
      if (motorTimeReached(nowUs, motorDueUs[activeMotorIndex])) {
        const uint32_t latenessUs = nowUs - motorDueUs[activeMotorIndex];
        if (latenessUs > motorSlotLatenessWindowMaxUs) {
          motorSlotLatenessWindowMaxUs = latenessUs;
        }
        activeMotorSlotStartLatenessUs = latenessUs;
        motorTaskState = MotorTaskState::SEND_SPEED;
      }
      return;

    case MotorTaskState::SEND_SPEED:
      sendMotorSpeedCommand(nowMs);
      motorTaskState = MotorTaskState::WAIT_SPEED_RESPONSE;
      return;

    case MotorTaskState::WAIT_SPEED_RESPONSE:
      if (collectMotorResponse(nowMs)) {
        finishSpeedResponse(SpeedResponsePhase::ON_TIME, nowMs);
      } else if (static_cast<uint32_t>(nowMs - motorRequestStartedMs) >=
                 RS485_RESPONSE_TIMEOUT_MS) {
        if (motorRxByteCount >= DDSM115_FRAME_SIZE) {
          if (static_cast<uint32_t>(nowMs - motorRequestStartedMs) >=
              RS485_RESPONSE_TIMEOUT_MS + RS485_FRAME_QUIET_MS) {
            finishSpeedResponse(SpeedResponsePhase::ON_TIME, nowMs);
          }
        } else {
          ++motorStatus[activeMotorIndex].firstAttemptTimeoutCount;
          motorTaskState = MotorTaskState::WAIT_SPEED_GRACE;
        }
      }
      return;

    case MotorTaskState::WAIT_SPEED_GRACE:
      if (collectMotorResponse(nowMs)) {
        finishSpeedResponse(SpeedResponsePhase::LATE, nowMs);
      } else if (static_cast<uint32_t>(nowMs - motorRequestStartedMs) >=
                 RS485_SPEED_FIRST_ATTEMPT_TOTAL_MS) {
        if (motorRxByteCount >= DDSM115_FRAME_SIZE) {
          finishSpeedResponse(SpeedResponsePhase::LATE, nowMs);
        } else {
          motorTaskState = MotorTaskState::SEND_SPEED_RETRY;
        }
      }
      return;

    case MotorTaskState::SEND_SPEED_RETRY:
      retryMotorSpeedCommand(nowMs);
      motorTaskState = MotorTaskState::WAIT_SPEED_RETRY_RESPONSE;
      return;

    case MotorTaskState::WAIT_SPEED_RETRY_RESPONSE:
      if (collectMotorResponse(nowMs)) {
        finishSpeedResponse(SpeedResponsePhase::RETRY, nowMs);
      } else if (static_cast<uint32_t>(nowMs - motorRequestStartedMs) >=
                 RS485_RESPONSE_TIMEOUT_MS) {
        if (motorRxByteCount >= DDSM115_FRAME_SIZE) {
          finishSpeedResponse(SpeedResponsePhase::RETRY, nowMs);
        } else {
          ++motorStatus[activeMotorIndex].finalTimeoutCount;
          recordMotorCommunicationError(MotorReplyResult::TIMEOUT, false);
          finishSpeedTransaction(nowMs);
        }
      }
      return;

    case MotorTaskState::SEND_TEMPERATURE:
      sendMotorTemperatureQuery(nowMs);
      motorTaskState = MotorTaskState::WAIT_TEMPERATURE_RESPONSE;
      return;

    case MotorTaskState::WAIT_TEMPERATURE_RESPONSE:
      serviceTemperatureResponse(nowMs);
      return;

    case MotorTaskState::NEXT_MOTOR:
      temperatureQueryThisSlotStarted = false;
      advanceMotorCycleDeadline(activeMotorIndex);
      while (motorTimeReached(nowUs, motorDueUs[activeMotorIndex])) {
        ++motorSlotMissCount;
        advanceMotorCycleDeadline(activeMotorIndex);
      }
      activeMotorIndex = (activeMotorIndex + 1) % MOTOR_COUNT;
      if (activeMotorIndex == 0) {
        ++motorSnapshotSequence;
      }
      motorTaskState = MotorTaskState::IDLE;
      updateMotorRate(nowMs);
      return;
  }
#else
  (void)nowMs;
#endif
}

void motorTask(uint32_t nowMs) {
  const uint32_t startUs = micros();
  serviceMotorStateMachine(nowMs);
  motorTaskLastUs = micros() - startUs;
  if (motorTaskLastUs > motorTaskWindowMaxUs) {
    motorTaskWindowMaxUs = motorTaskLastUs;
  }
}

bool motorAllowsSlowTasks() {
#if PICO_B_ENABLE_MOTORS
  return motorTaskState == MotorTaskState::IDLE ||
         motorTaskState == MotorTaskState::NEXT_MOTOR;
#else
  return true;
#endif
}

bool createEntities() {
  zeroEntities();
  rosSessionError = false;
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) {
    return false;
  }
  if (rclc_node_init_default(&node, NODE_NAME, "", &support) != RCL_RET_OK) {
    return false;
  }
  if (initBestEffortPublisher(
          heartbeatPublisher,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
          HEARTBEAT_TOPIC) != RCL_RET_OK) {
    return false;
  }
#if PICO_B_ENABLE_ROS_SBUS_PUBLISHER
  if (initBestEffortPublisher(
          sbusPublisher,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
          SBUS_TOPIC) != RCL_RET_OK) {
    return false;
  }
#endif
#if PICO_B_ENABLE_ROS_BATTERY_PUBLISHER
  if (initBestEffortPublisher(
          batteryPublisher,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          BATTERY_TOPIC) != RCL_RET_OK) {
    return false;
  }
#endif
#if PICO_B_ENABLE_ROS_MOTOR_PUBLISHER
  if (initBestEffortPublisher(
          motorsRpmPublisher,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
          MOTORS_RPM_TOPIC) != RCL_RET_OK) {
    return false;
  }
#endif
#if PICO_B_ENABLE_ROS_DIAGNOSTICS
  if (initBestEffortPublisher(
          diagnosticPublisher,
          ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
          DIAGNOSTICS_TOPIC) != RCL_RET_OK) {
    return false;
  }
#endif

#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
  {
    rcl_subscription_options_t subOpts = rcl_subscription_get_default_options();
    subOpts.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    subOpts.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    subOpts.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    subOpts.qos.depth = 1;
    if (rcl_subscription_init(&cmdVelSubscriber,
                              &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                              CMD_VEL_TOPIC,
                              &subOpts) != RCL_RET_OK) {
      return false;
    }
    if (rclc_executor_init(&cmdVelExecutor,
                           &support.context,
                           CMD_VEL_EXECUTOR_HANDLE_COUNT,
                           &allocator) != RCL_RET_OK) {
      return false;
    }
    if (rclc_executor_add_subscription(&cmdVelExecutor,
                                       &cmdVelSubscriber,
                                       &cmdVelMsg,
                                       &cmdVelCallback,
                                       ON_NEW_DATA) != RCL_RET_OK) {
      return false;
    }
    cmdVelSubscriberCreated = true;
  }
#endif

#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
  {
    rcl_subscription_options_t subOpts = rcl_subscription_get_default_options();
    subOpts.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    subOpts.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    subOpts.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    subOpts.qos.depth = 1;
    if (rcl_subscription_init(
            &wheelCommandSubscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(lessien_interfaces, msg, WheelCommand),
            WHEEL_COMMAND_TOPIC,
            &subOpts) != RCL_RET_OK) {
      return false;
    }
    wheelCommandSubscriberInitialized = true;
    if (rclc_executor_init(&wheelCommandExecutor,
                           &support.context,
                           WHEEL_COMMAND_EXECUTOR_HANDLE_COUNT,
                           &allocator) != RCL_RET_OK) {
      return false;
    }
    wheelCommandExecutorInitialized = true;
    if (rclc_executor_add_subscription(&wheelCommandExecutor,
                                       &wheelCommandSubscriber,
                                       &wheelCommandMsg,
                                       &wheelCommandCallback,
                                       ON_NEW_DATA) != RCL_RET_OK) {
      return false;
    }
    const rmw_ret_t syncRet =
        rmw_uros_sync_session(WHEEL_COMMAND_TIME_SYNC_TIMEOUT_MS);
    wheelCommandStatus.sourceTimeMapped =
        syncRet == RMW_RET_OK && rmw_uros_epoch_synchronized();
    wheelCommandStatus.sourceTimeTrusted =
        wheelCommandStatus.sourceTimeMapped &&
        (PICO_B_SOURCE_TIME_MAPPING_VERIFIED == 1);
  }
#endif

  const uint32_t nowMs = millis();
  heartbeatMsg.data = heartbeatSequence;
  lastHeartbeatMs = nowMs;
  lastSbusPublishMs = nowMs;
  lastBatteryPublishMs = nowMs;
  lastMotorsRpmPublishSequence = motorSnapshotSequence;
  lastDiagnosticPublishMs = nowMs;
  lastDiagnosticPageMs = nowMs;
  lastRunningAgentPingMs = nowMs;
  runningAgentPingFailures = 0;
  entitiesCreated = true;
  return true;
}

uint8_t calculateBatterySoc(float voltage) {
  float percent = (voltage - BATTERY_EMPTY_VOLTAGE) * 100.0f /
                  (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE);
  percent = constrain(percent, 0.0f, 100.0f);
  return static_cast<uint8_t>(percent + 0.5f);
}

void measureBattery(uint32_t nowMs) {
#if PICO_B_ENABLE_BATTERY_ADC
  uint32_t rawTotal = 0;
  for (uint8_t i = 0; i < BATTERY_ADC_SAMPLE_COUNT; ++i) {
    rawTotal += analogRead(BATTERY_ADC_PIN);
  }

  const float averageRaw = rawTotal / static_cast<float>(BATTERY_ADC_SAMPLE_COUNT);
  const float adcVoltage = averageRaw * ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE;
  const float measuredBatteryVoltage = adcVoltage * BATTERY_DIVIDER_RATIO *
                                       BATTERY_CALIBRATION_FACTOR;

  batteryStatus.averageRawAdc = averageRaw;
  batteryStatus.adcVoltage = adcVoltage;
  batteryStatus.sampleCount += BATTERY_ADC_SAMPLE_COUNT;

  if (!isfinite(measuredBatteryVoltage) ||
      measuredBatteryVoltage < BATTERY_VALID_MIN_VOLTAGE ||
      measuredBatteryVoltage > BATTERY_VALID_MAX_VOLTAGE) {
    batteryStatus.valid = false;
    batteryStatus.low = false;
    batteryStatus.critical = false;
    ++batteryStatus.adcErrorCount;
    return;
  }

  if (!batteryFilterInitialized) {
    batteryStatus.filteredVoltage = measuredBatteryVoltage;
    batteryFilterInitialized = true;
  } else {
    batteryStatus.filteredVoltage += BATTERY_EMA_ALPHA *
                                     (measuredBatteryVoltage -
                                      batteryStatus.filteredVoltage);
  }

  batteryStatus.millivolts = static_cast<uint32_t>(
      batteryStatus.filteredVoltage * 1000.0f + 0.5f);
  batteryStatus.socPercent = calculateBatterySoc(batteryStatus.filteredVoltage);
  batteryStatus.lastUpdateMs = nowMs;
  batteryStatus.valid = true;
  batteryStatus.low = batteryStatus.filteredVoltage <= BATTERY_LOW_VOLTAGE;
  batteryStatus.critical =
      batteryStatus.filteredVoltage <= BATTERY_CRITICAL_VOLTAGE;
#else
  (void)nowMs;
#endif
}

void setSbusChannelsToSafeNeutral() {
  for (size_t channel = 0; channel < SBUS_CHANNEL_COUNT; ++channel) {
    sbusStatus.channels[channel] = SBUS_SAFE_NEUTRAL;
  }
  sbusStatus.digitalChannel17 = false;
  sbusStatus.digitalChannel18 = false;
}

void setSbusSafeState() {
  setSbusChannelsToSafeNeutral();
  sbusStatus.frameLost = true;
  sbusStatus.failsafe = true;
  sbusStatus.connected = false;
  sbusStatus.lastFrameValid = false;
  sbusStatus.frameRateHz = 0.0f;
  sbusRateWindowStartMs = 0;
  sbusRateWindowFrameCount = 0;
}

bool decodeSbusFrame(const uint8_t *frame, uint32_t nowMs) {
  if (frame[0] != SBUS_START_BYTE ||
      frame[SBUS_FRAME_SIZE - 1] != SBUS_END_BYTE) {
    ++sbusStatus.invalidFrameCount;
    sbusStatus.lastFrameValid = false;
    return false;
  }

  uint16_t decodedChannels[SBUS_CHANNEL_COUNT];
  for (uint8_t channel = 0; channel < SBUS_CHANNEL_COUNT; ++channel) {
    const uint16_t bitIndex = static_cast<uint16_t>(channel) * 11;
    const uint8_t byteIndex = 1 + bitIndex / 8;
    const uint8_t bitOffset = bitIndex % 8;
    const uint32_t packed = static_cast<uint32_t>(frame[byteIndex]) |
                            (static_cast<uint32_t>(frame[byteIndex + 1]) << 8) |
                            (static_cast<uint32_t>(frame[byteIndex + 2]) << 16);
    decodedChannels[channel] = (packed >> bitOffset) & 0x07FF;
  }

  const uint8_t flags = frame[23];
  for (uint8_t channel = 0; channel < SBUS_CHANNEL_COUNT; ++channel) {
    sbusStatus.channels[channel] = decodedChannels[channel];
  }
  sbusStatus.digitalChannel17 = (flags & 0x01) != 0;
  sbusStatus.digitalChannel18 = (flags & 0x02) != 0;
  sbusStatus.frameLost = (flags & 0x04) != 0;
  sbusStatus.failsafe = (flags & 0x08) != 0;
  if (sbusStatus.frameLost) {
    ++sbusStatus.frameLostCount;
  }
  if (sbusStatus.failsafe) {
    setSbusChannelsToSafeNeutral();
  }

  if (sbusRateWindowFrameCount == 0) {
    sbusRateWindowStartMs = nowMs;
  }
  ++sbusRateWindowFrameCount;
  const uint32_t rateWindowElapsedMs = nowMs - sbusRateWindowStartMs;
  if (rateWindowElapsedMs >= SBUS_RATE_WINDOW_MS) {
    sbusStatus.frameRateHz =
        sbusRateWindowFrameCount * 1000.0f / rateWindowElapsedMs;
    sbusRateWindowFrameCount = 0;
  }

  sbusStatus.lastFrameMs = nowMs;
  ++sbusStatus.validFrameCount;
  sbusStatus.connected = !sbusStatus.failsafe;
  sbusStatus.lastFrameValid = true;
  return true;
}

void sbusTask(uint32_t nowMs) {
#if PICO_B_ENABLE_SBUS
  if (Serial2.overflow()) {
    ++sbusStatus.invalidFrameCount;
    sbusStatus.lastFrameValid = false;
    sbusFrameIndex = 0;
  }

  while (Serial2.available() > 0) {
    const int received = Serial2.read();
    if (received < 0) {
      break;
    }

    const uint8_t value = static_cast<uint8_t>(received);
    if (sbusFrameIndex == 0) {
      if (value != SBUS_START_BYTE) {
        ++sbusStatus.invalidFrameCount;
        sbusStatus.lastFrameValid = false;
        continue;
      }
      sbusFrameBuffer[sbusFrameIndex++] = value;
      continue;
    }

    sbusFrameBuffer[sbusFrameIndex++] = value;
    if (sbusFrameIndex == SBUS_FRAME_SIZE) {
      decodeSbusFrame(sbusFrameBuffer, nowMs);
      sbusFrameIndex = 0;
    }
  }

  if (sbusStatus.connected &&
      static_cast<uint32_t>(nowMs - sbusStatus.lastFrameMs) >=
          SBUS_TIMEOUT_MS) {
    sbusFrameIndex = 0;
    setSbusSafeState();
  }
#else
  (void)nowMs;
#endif
}

bool publishHeartbeat(uint32_t nowMs) {
  if (!periodElapsed(nowMs, lastHeartbeatMs, HEARTBEAT_PERIOD_MS)) {
    return true;
  }

  heartbeatMsg.data = ++heartbeatSequence;
  const rcl_ret_t ret = rcl_publish(&heartbeatPublisher, &heartbeatMsg, nullptr);
  if (ret != RCL_RET_OK) {
    ++heartbeatPublishError;
    rosSessionError = true;
    return false;
  }
  return true;
}

bool publishSbus(uint32_t nowMs) {
#if PICO_B_ENABLE_ROS_SBUS_PUBLISHER
  if (!periodElapsed(nowMs, lastSbusPublishMs, SBUS_PUBLISH_PERIOD_MS)) {
    return true;
  }

  for (size_t channel = 0; channel < SBUS_CHANNEL_COUNT; ++channel) {
    sbusMessageData[channel] = static_cast<int16_t>(sbusStatus.channels[channel]);
  }
  const rcl_ret_t ret = rcl_publish(&sbusPublisher, &sbusMsg, nullptr);
  if (ret != RCL_RET_OK) {
    ++telemetryPublishError;
    rosSessionError = true;
    return false;
  }
#else
  (void)nowMs;
#endif
  return true;
}

bool publishBattery(uint32_t nowMs) {
#if PICO_B_ENABLE_ROS_BATTERY_PUBLISHER
  if (!periodElapsed(nowMs, lastBatteryPublishMs, BATTERY_PUBLISH_PERIOD_MS)) {
    return true;
  }
  if (!batteryStatus.valid || !isfinite(batteryStatus.filteredVoltage)) {
    return true;
  }

  batteryMsg.data = batteryStatus.filteredVoltage;
  const rcl_ret_t ret = rcl_publish(&batteryPublisher, &batteryMsg, nullptr);
  if (ret != RCL_RET_OK) {
    ++telemetryPublishError;
    rosSessionError = true;
    return false;
  }
#else
  (void)nowMs;
#endif
  return true;
}

bool publishMotorsRpm(uint32_t nowMs) {
#if PICO_B_ENABLE_ROS_MOTOR_PUBLISHER
  if (lastMotorsRpmPublishSequence == motorSnapshotSequence) {
    return true;
  }
  lastMotorsRpmPublishSequence = motorSnapshotSequence;

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    motorsRpmMessageData[i] =
        motorTelemetryFresh(i, nowMs) ? motorStatus[i].measuredRpm : 0;
  }
  const rcl_ret_t ret = rcl_publish(&motorsRpmPublisher, &motorsRpmMsg, nullptr);
  if (ret != RCL_RET_OK) {
    ++telemetryPublishError;
    rosSessionError = true;
    return false;
  }
#else
  (void)nowMs;
#endif
  return true;
}

void rotateDiagnosticPage(uint32_t nowMs) {
  if (!periodElapsed(nowMs, lastDiagnosticPageMs, DIAGNOSTIC_PAGE_PERIOD_MS)) {
    return;
  }

  switch (diagnosticPage) {
    case DiagnosticPage::SBUS:
      diagnosticPage = DiagnosticPage::BATTERY;
      return;
    case DiagnosticPage::BATTERY:
      diagnosticPage = DiagnosticPage::MICRO_ROS;
      return;
    case DiagnosticPage::MICRO_ROS:
      diagnosticPage = DiagnosticPage::OLED;
      return;
    case DiagnosticPage::OLED:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTORS;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTORS:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_1_FL;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_1_FL:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_2_RL;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_2_RL:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_3_FR;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_3_FR:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_4_RR;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_4_RR:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_COMM_12;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_COMM_12:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_COMM_34;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_COMM_34:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_DETAIL_1;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_DETAIL_1:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_DETAIL_2;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_DETAIL_2:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_DETAIL_3;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_DETAIL_3:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_DETAIL_4;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_DETAIL_4:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_LATENCY_12;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_LATENCY_12:
#if PICO_B_ENABLE_MOTORS
      diagnosticPage = DiagnosticPage::MOTOR_LATENCY_34;
#else
      diagnosticPage = DiagnosticPage::SBUS;
#endif
      return;
    case DiagnosticPage::MOTOR_LATENCY_34:
      diagnosticPage = DiagnosticPage::MOTOR_TEMPERATURE_SCHEDULE;
      return;
    case DiagnosticPage::MOTOR_TEMPERATURE_SCHEDULE:
      diagnosticPage = DiagnosticPage::MOTOR_SCHEDULER;
      return;
    case DiagnosticPage::MOTOR_SCHEDULER:
      diagnosticPage = DiagnosticPage::MOTOR_TRANSACTION_GUARD;
      return;
    case DiagnosticPage::MOTOR_TRANSACTION_GUARD:
      diagnosticPage = DiagnosticPage::OLED_CONTROL;
      return;
    case DiagnosticPage::OLED_CONTROL:
      diagnosticPage = DiagnosticPage::RELAYS;
      return;
    case DiagnosticPage::RELAYS:
      diagnosticPage = DiagnosticPage::CAMERA_SERVO;
      return;
    case DiagnosticPage::CAMERA_SERVO:
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
      diagnosticPage = DiagnosticPage::WHEEL_COMMAND;
#else
      diagnosticPage = DiagnosticPage::CMD_VEL;
#endif
      return;
    case DiagnosticPage::CMD_VEL:
      diagnosticPage = DiagnosticPage::SBUS;
      return;
    case DiagnosticPage::WHEEL_COMMAND:
      diagnosticPage = DiagnosticPage::WHEEL_FRESHNESS;
      return;
    case DiagnosticPage::WHEEL_FRESHNESS:
      diagnosticPage = DiagnosticPage::SBUS;
      return;
  }
}

uint32_t sbusFrameAgeMs(uint32_t nowMs) {
  if (!sbusStatus.connected && sbusStatus.lastFrameMs == 0) {
    return 0;
  }
  return nowMs - sbusStatus.lastFrameMs;
}

const char *rosStateText() {
  switch (rosState) {
    case RosState::WAIT_AGENT:
      return "WAIT_AGENT";
    case RosState::CREATE_ENTITIES:
      return "CREATE_ENTITIES";
    case RosState::RUNNING:
      return "RUNNING";
    case RosState::DESTROY_ENTITIES:
      return "DESTROY_ENTITIES";
  }
  return "UNKNOWN";
}

void fillSbusDiagnostic(uint32_t nowMs,
                        diagnostic_msgs__msg__DiagnosticStatus &status) {
  const bool unhealthy = !sbusStatus.connected || sbusStatus.failsafe;
  status.level = unhealthy
                     ? static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN)
                     : static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  setLiteralString(status.name, "pico_b/sbus");
  setLiteralString(status.message, unhealthy ? "SBUS_UNHEALTHY" : "OK");
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueFloat(0, "sbusHz", sbusStatus.frameRateHz, 1);
  setDiagnosticValueBool(1, "connected", sbusStatus.connected);
  setDiagnosticValueBool(2, "failsafe", sbusStatus.failsafe);
  setDiagnosticValueUnsigned(3, "frameLostCount", sbusStatus.frameLostCount);
  setDiagnosticValueUnsigned(4, "invalidFrameCount", sbusStatus.invalidFrameCount);
  setDiagnosticValueUnsigned(5, "frameAgeMs", sbusFrameAgeMs(nowMs));
}

void fillBatteryDiagnostic(diagnostic_msgs__msg__DiagnosticStatus &status) {
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  const char *message = "OK";
  if (!batteryStatus.valid) {
    status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "BATTERY_INVALID";
  } else if (batteryStatus.critical) {
    status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__ERROR);
    message = "BATTERY_CRITICAL";
  } else if (batteryStatus.low) {
    status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "BATTERY_LOW";
  }
  setLiteralString(status.name, "pico_b/battery");
  setLiteralString(status.message, message);
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueFloat(0, "voltage", batteryStatus.filteredVoltage, 2);
  setDiagnosticValueUnsigned(1, "socPercent", batteryStatus.socPercent);
  setDiagnosticValueBool(2, "valid", batteryStatus.valid);
  setDiagnosticValueBool(3, "low", batteryStatus.low);
  setDiagnosticValueBool(4, "critical", batteryStatus.critical);
  setDiagnosticValueUnsigned(5, "adcRaw",
                             static_cast<uint32_t>(batteryStatus.averageRawAdc + 0.5f));
}

void fillMicroRosDiagnostic(diagnostic_msgs__msg__DiagnosticStatus &status) {
  const bool publishClean = heartbeatPublishError == 0 &&
                            telemetryPublishError == 0 &&
                            diagnosticPublishError == 0;
  status.level = publishClean
                     ? static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK)
                     : static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
  setLiteralString(status.name, "pico_b/micro_ros");
  setLiteralString(status.message, publishClean ? "OK" : "PUBLISH_ERROR");
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValue(0, "agentState", rosStateText());
  setDiagnosticValueUnsigned(1, "heartbeatPublishError", heartbeatPublishError);
  setDiagnosticValueUnsigned(2, "telemetryPublishError", telemetryPublishError);
  setDiagnosticValueUnsigned(3, "diagnosticPublishError", diagnosticPublishError);
  setDiagnosticValueUnsigned(4, "reconnectCount", reconnectCount);
  setDiagnosticValue(5, "bootStage", "B3A");
}

void fillOledDiagnostic(diagnostic_msgs__msg__DiagnosticStatus &status) {
  const bool enabled = PICO_B_ENABLE_OLED != 0;
  const bool ok = !enabled || oledAvailable;
  status.level = ok ? static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK)
                    : static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
  setLiteralString(status.name, "pico_b/oled");
  setLiteralString(status.message, ok ? "OK" : "OLED_UNAVAILABLE");
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueBool(0, "oledEnabled", enabled);
  setDiagnosticValueBool(1, "oledAvailable", oledAvailable);
  setDiagnosticValueFloat(2, "oledUpdateHz", oledWindowHz, 1);
  setDiagnosticValueUnsigned(3, "oledLastUpdateUs", oledLastUpdateUs);
  setDiagnosticValueUnsigned(4, "oledWindowMaxUs", oledWindowMaxUs);
  setDiagnosticValueUnsigned(5, "oledInitFailureCount", oledInitFailureCount);
}

void fillOledControlDiagnostic(uint32_t nowMs,
                               diagnostic_msgs__msg__DiagnosticStatus &status) {
  const bool manual = PICO_B_ENABLE_MANUAL_OLED_PAGING != 0;
  const bool sbusHealthy = sbusRealHealthyFrame(nowMs);
  const uint16_t ch3Value = sbusStatus.channels[2];
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  setLiteralString(status.name, "pico_b/oled_control");
  setLiteralString(status.message, manual ? "MANUAL" : "AUTO");
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueUnsigned(0, "pageIndex", static_cast<uint8_t>(oledPage));
  setDiagnosticValue(1, "pageName", oledPageName());
  setDiagnosticValueUnsigned(2, "pageChangeCount", oledPageChangeCount);
  setDiagnosticValueUnsigned(3, "ch3Value", ch3Value);
  setDiagnosticValueBool(4, "inputArmed", oledInputArmed);
  setDiagnosticValueBool(5, "sbusHealthy", sbusHealthy);
}

uint8_t motorOnlineMask(uint32_t nowMs) {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (motorTelemetryFresh(i, nowMs)) {
      mask |= static_cast<uint8_t>(1U << i);
    }
  }
  return mask;
}

uint8_t motorFaultMask() {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (motorStatus[i].errorCode != 0) {
      mask |= static_cast<uint8_t>(1U << i);
    }
  }
  return mask;
}

bool motorCommunicationWarningPresent() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const MotorStatus &status = motorStatus[i];
    if (status.online &&
        (status.crcErrorCount != 0 || status.timeoutCount != 0 ||
         status.wrongIdCount != 0 || status.lengthErrorCount != 0)) {
      return true;
    }
  }
  return false;
}

void fillMotorsDiagnostic(uint32_t nowMs,
                          diagnostic_msgs__msg__DiagnosticStatus &status) {
  const uint8_t onlineMask = motorOnlineMask(nowMs);
  const uint8_t faultMask = motorFaultMask();
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  const char *message = "OK";
  if (faultMask != 0) {
    status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__ERROR);
    message = "MOTOR_FAULT";
  } else if (onlineMask != 0x0F) {
    status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__ERROR);
    message = "MOTOR_OFFLINE";
  } else if (motorCommunicationWarningPresent()) {
    status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "MOTOR_COMM_WARNING";
  }

  setLiteralString(status.name, "pico_b/motors");
  setLiteralString(status.message, message);
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueFloat(0, "motorTransactionHz", motorTransactionHz, 1);
  setDiagnosticValueFloat(1, "motorCycleHz", motorCycleHz, 1);
  setDiagnosticValueHex8(2, "onlineMask", onlineMask);
  setDiagnosticValueHex8(3, "faultMask", faultMask);
  setDiagnosticValueSigned(4, "commandRpm", MOTOR_TARGET_RPM);
  setDiagnosticValueUnsigned(5, "motorTaskWindowMaxUs", motorTaskWindowMaxUs);
}

void fillSingleMotorDiagnostic(uint8_t motorIndex, uint32_t nowMs,
                               diagnostic_msgs__msg__DiagnosticStatus &status) {
  static const char *const names[MOTOR_COUNT] = {
    "pico_b/motor_1_fl",
    "pico_b/motor_2_rl",
    "pico_b/motor_3_fr",
    "pico_b/motor_4_rr",
  };
  const MotorStatus &motor = motorStatus[motorIndex];
  const bool fresh = motorTelemetryFresh(motorIndex, nowMs);
  status.level = fresh && motor.errorCode == 0
                     ? static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK)
                     : static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__ERROR);
  setLiteralString(status.name, names[motorIndex]);
  setLiteralString(status.message,
                   fresh ? (motor.errorCode == 0 ? "OK" : "MOTOR_FAULT")
                         : "MOTOR_OFFLINE");
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueBool(0, "online", fresh);
  setDiagnosticValueSigned(1, "rpm", fresh ? motor.measuredRpm : 0);
  setDiagnosticValueFloat(2, "currentA", fresh ? motor.currentA : 0.0f, 2);
  setDiagnosticValueUnsigned(3, "temperatureC",
                             fresh && motor.temperatureValid ? motor.temperatureC : 0);
  setDiagnosticValueHex8(4, "errorHex", motor.errorCode);
  setDiagnosticValueHex8(5, "mode", motor.mode);
}

void fillMotorCommDiagnostic(uint8_t firstMotorIndex,
                             diagnostic_msgs__msg__DiagnosticStatus &status) {
  const uint8_t secondMotorIndex = firstMotorIndex + 1;
  const bool firstPair = firstMotorIndex == 0;
  setLiteralString(status.name,
                   firstPair ? "pico_b/motor_comm_12" : "pico_b/motor_comm_34");
  setLiteralString(status.message, "OK");
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueUnsigned(0, firstPair ? "M1_crc" : "M3_crc",
                             motorStatus[firstMotorIndex].crcErrorCount);
  setDiagnosticValueUnsigned(1, firstPair ? "M1_timeout" : "M3_timeout",
                             motorStatus[firstMotorIndex].timeoutCount);
  setDiagnosticValueUnsigned(2, firstPair ? "M1_wrongId" : "M3_wrongId",
                             motorStatus[firstMotorIndex].wrongIdCount);
  setDiagnosticValueUnsigned(3, firstPair ? "M2_crc" : "M4_crc",
                             motorStatus[secondMotorIndex].crcErrorCount);
  setDiagnosticValueUnsigned(4, firstPair ? "M2_timeout" : "M4_timeout",
                             motorStatus[secondMotorIndex].timeoutCount);
  setDiagnosticValueUnsigned(5, firstPair ? "M2_wrongId" : "M4_wrongId",
                             motorStatus[secondMotorIndex].wrongIdCount);
}

void fillMotorDetailDiagnostic(uint8_t motorIndex,
                               diagnostic_msgs__msg__DiagnosticStatus &status) {
  static const char *const names[MOTOR_COUNT] = {
    "pico_b/motor_detail_1_fl",
    "pico_b/motor_detail_2_rl",
    "pico_b/motor_detail_3_fr",
    "pico_b/motor_detail_4_rr",
  };
  const MotorStatus &motor = motorStatus[motorIndex];
  setLiteralString(status.name, names[motorIndex]);
  setLiteralString(status.message, "OK");
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueUnsigned(0, "speedTimeout", motor.speedTimeoutCount);
  setDiagnosticValueUnsigned(1, "temperatureTimeout",
                             motor.temperatureTimeoutCount);
  setDiagnosticValueUnsigned(2, "speedCrcError", motor.speedCrcErrorCount);
  setDiagnosticValueUnsigned(3, "temperatureCrcError",
                             motor.temperatureCrcErrorCount);
  setDiagnosticValueUnsigned(4, "rxLengthLast", motor.rxLengthLast);
  setDiagnosticValueUnsigned(5, "lateResponseCount", motor.lateResponseCount);
}

void fillMotorLatencyDiagnostic(uint8_t firstMotorIndex,
                                diagnostic_msgs__msg__DiagnosticStatus &status) {
  const uint8_t secondMotorIndex = firstMotorIndex + 1;
  const bool firstPair = firstMotorIndex == 0;
  setLiteralString(status.name,
                   firstPair ? "pico_b/motor_latency_12"
                             : "pico_b/motor_latency_34");
  setLiteralString(status.message, "OK");
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueUnsigned(0, firstPair ? "M1_latencyLastUs"
                                          : "M3_latencyLastUs",
                             motorStatus[firstMotorIndex].responseLatencyLastUs);
  setDiagnosticValueUnsigned(1, firstPair ? "M1_latencyMaxUs"
                                          : "M3_latencyMaxUs",
                             motorStatus[firstMotorIndex].responseLatencyWindowMaxUs);
  setDiagnosticValueUnsigned(2, firstPair ? "M1_rxLengthLast"
                                          : "M3_rxLengthLast",
                             motorStatus[firstMotorIndex].rxLengthLast);
  setDiagnosticValueUnsigned(3, firstPair ? "M2_latencyLastUs"
                                          : "M4_latencyLastUs",
                             motorStatus[secondMotorIndex].responseLatencyLastUs);
  setDiagnosticValueUnsigned(4, firstPair ? "M2_latencyMaxUs"
                                          : "M4_latencyMaxUs",
                             motorStatus[secondMotorIndex].responseLatencyWindowMaxUs);
  setDiagnosticValueUnsigned(5, firstPair ? "M2_rxLengthLast"
                                          : "M4_rxLengthLast",
                             motorStatus[secondMotorIndex].rxLengthLast);
}

void fillMotorTemperatureScheduleDiagnostic(
    diagnostic_msgs__msg__DiagnosticStatus &status) {
  setLiteralString(status.name, "pico_b/motor_temperature_schedule");
  setLiteralString(status.message, "OK");
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueFloat(0, "temperatureQueryHz",
                          motorTemperatureQueryHz, 1);
  setDiagnosticValueUnsigned(1, "M1_tempQueryCount",
                             motorTemperatureQueryCount[0]);
  setDiagnosticValueUnsigned(2, "M2_tempQueryCount",
                             motorTemperatureQueryCount[1]);
  setDiagnosticValueUnsigned(3, "M3_tempQueryCount",
                             motorTemperatureQueryCount[2]);
  setDiagnosticValueUnsigned(4, "M4_tempQueryCount",
                             motorTemperatureQueryCount[3]);
  setDiagnosticValueUnsigned(5, "tempScheduleMissCount",
                             motorTemperatureScheduleMissCount);
}

void fillMotorSchedulerDiagnostic(
    diagnostic_msgs__msg__DiagnosticStatus &status) {
  setLiteralString(status.name, "pico_b/motor_scheduler");
  setLiteralString(status.message, "OK");
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueUnsigned(0, "configuredCycleHz", PICO_B_MOTOR_CYCLE_HZ);
  setDiagnosticValueFloat(1, "motorCycleHz", motorCycleHz, 1);
  setDiagnosticValueFloat(2, "motorTransactionHz", motorTransactionHz, 1);
  setDiagnosticValueUnsigned(3, "motorSlotPeriodUs", MOTOR_SLOT_PERIOD_US);
  setDiagnosticValueUnsigned(4, "motorSlotMissCount", motorSlotMissCount);
  setDiagnosticValueUnsigned(5, "motorSlotLatenessMaxUs",
                             motorSlotLatenessWindowMaxUs);
}

void fillMotorTransactionGuardDiagnostic(
    diagnostic_msgs__msg__DiagnosticStatus &status) {
  setLiteralString(status.name, "pico_b/motor_transaction_guard");
  setLiteralString(status.message, "OK");
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueUnsigned(0, "tempStartCount", motorTempStartCount);
  setDiagnosticValueUnsigned(1, "tempDeferredCount", motorTempDeferredCount);
  setDiagnosticValueUnsigned(2, "tempGuardRejectCount", motorTempGuardRejectCount);
  setDiagnosticValueUnsigned(3, "tempStartLatenessMaxUs",
                             motorTempStartLatenessWindowMaxUs);
  setDiagnosticValueUnsigned(4, "speedConsecutiveFailMax",
                             speedConsecutiveFailMax);
  setDiagnosticValueUnsigned(5, "lastOfflineMotorId", lastOfflineMotorId);
}

void fillRelaysDiagnostic(diagnostic_msgs__msg__DiagnosticStatus &status) {
#if PICO_B_ENABLE_RELAYS
  const bool ok = relayOutputSafe || relayInhibitCode == RELAY_INHIBIT_REARM;
  status.level = ok ? static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK)
                    : static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
  setLiteralString(status.name, "pico_b/relays");
  setLiteralString(status.message, ok ? "OK" : "RELAY_INHIBIT");
#else
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  setLiteralString(status.name, "pico_b/relays");
  setLiteralString(status.message, "DISABLED");
#endif
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueBool(0, "relay1On", relayStatus[0].outputOn);
  setDiagnosticValueBool(1, "relay2On", relayStatus[1].outputOn);
  setDiagnosticValueBool(2, "relay1Requested", relayStatus[0].requestedOn);
  setDiagnosticValueBool(3, "relay2Requested", relayStatus[1].requestedOn);
  setDiagnosticValueBool(4, "outputSafe", relayOutputSafe);
  setDiagnosticValueUnsigned(5, "relayFailsafeCount", relayFailsafeCount);
}

void fillCameraServoDiagnostic(
    diagnostic_msgs__msg__DiagnosticStatus &status) {
#if PICO_B_ENABLE_CAMERA_SERVO
  const bool ok = cameraServoStatus.attached;
  status.level = ok ? static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK)
                    : static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
  setLiteralString(status.name, "pico_b/camera_servo");
  setLiteralString(status.message, ok ? "OK" : "SERVO_DETACHED");
#else
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  setLiteralString(status.name, "pico_b/camera_servo");
  setLiteralString(status.message, "DISABLED");
#endif
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueBool(0, "attached", cameraServoStatus.attached);
  setDiagnosticValueSigned(1, "targetAngleDeg",
                           cameraServoStatus.targetAbsoluteAngleDeg);
  setDiagnosticValueSigned(
      2, "currentAngleDeg",
      static_cast<int16_t>((cameraServoStatus.currentAngleTenthsDeg + 5) / 10));
  setDiagnosticValueUnsigned(3, "pulseUs", cameraServoStatus.currentPulseUs);
  setDiagnosticValueBool(4, "inputHealthy", cameraServoStatus.inputHealthy);
  setDiagnosticValueUnsigned(5, "servoFailsafeCount",
                             cameraServoFailsafeCount);
}

void fillCmdVelDiagnostic(uint32_t nowMs,
                          diagnostic_msgs__msg__DiagnosticStatus &status) {
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
  const bool neverReceived = (cmdVelStatus.receiveCount == 0);
  uint32_t commandAgeMs = 0;
  if (!neverReceived) {
    commandAgeMs = nowMs - cmdVelStatus.lastReceiveMs;
  }
  const bool finite = cmdVelStatus.commandFinite;
  const bool fresh = cmdVelStatus.commandFresh;

  uint8_t level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  const char *message = "CMD_VEL_RX_ONLY";
  if (neverReceived) {
    level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "CMD_VEL_NEVER_RECEIVED";
  } else if (!finite) {
    level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "CMD_VEL_INVALID";
  } else if (!fresh) {
    level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "CMD_VEL_STALE";
  }

  status.level = level;
  setLiteralString(status.name, CMD_VEL_DIAG_NAME);
  setLiteralString(status.message, message);
  status.values.size = DIAG_VALUE_CAPACITY;

  const float lx = finite ? cmdVelStatus.linearX : 0.0f;
  const float az = finite ? cmdVelStatus.angularZ : 0.0f;
  setDiagnosticValueFloat(0, "linearX", lx, 2);
  setDiagnosticValueFloat(1, "angularZ", az, 2);
  setDiagnosticValueUnsigned(2, "receiveCount", cmdVelStatus.receiveCount);
  setDiagnosticValueUnsigned(3, "commandAgeMs", commandAgeMs);
  setDiagnosticValueBool(4, "commandFresh", fresh);
  setDiagnosticValueUnsigned(5, "rejectCount", cmdVelStatus.rejectCount);
#else
  status.level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  setLiteralString(status.name, CMD_VEL_DIAG_NAME);
  setLiteralString(status.message, "DISABLED");
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueFloat(0, "linearX", 0.0f, 2);
  setDiagnosticValueFloat(1, "angularZ", 0.0f, 2);
  setDiagnosticValueUnsigned(2, "receiveCount", 0);
  setDiagnosticValueUnsigned(3, "commandAgeMs", 0);
  setDiagnosticValueBool(4, "commandFresh", false);
  setDiagnosticValueUnsigned(5, "rejectCount", 0);
#endif
}

void fillWheelCommandDiagnostic(
    diagnostic_msgs__msg__DiagnosticStatus &status) {
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
  const bool structuralValid = wheelCommandStatus.hasCurrentSessionSample;
  status.level = wheelCommandStatus.commandFresh
                     ? static_cast<uint8_t>(
                           diagnostic_msgs__msg__DiagnosticStatus__OK)
                     : static_cast<uint8_t>(
                           diagnostic_msgs__msg__DiagnosticStatus__WARN);
  setLiteralString(status.name, WHEEL_COMMAND_DIAG_NAME);
  setLiteralString(status.message,
                   wheelCommandStatus.commandFresh
                       ? "RX_ONLY_TIME_OK"
                       : (wheelCommandStatus.localFresh
                              ? "RX_ONLY_TIME_UNVERIFIED"
                              : "RX_ONLY_INVALID"));
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueFloat(
      0, "flRadS",
      structuralValid ? static_cast<float>(wheelCommandStatus.velocityRadS[0])
                      : 0.0f,
      2);
  setDiagnosticValueFloat(
      1, "rlRadS",
      structuralValid ? static_cast<float>(wheelCommandStatus.velocityRadS[1])
                      : 0.0f,
      2);
  setDiagnosticValueFloat(
      2, "frRadS",
      structuralValid ? static_cast<float>(wheelCommandStatus.velocityRadS[2])
                      : 0.0f,
      2);
  setDiagnosticValueFloat(
      3, "rrRadS",
      structuralValid ? static_cast<float>(wheelCommandStatus.velocityRadS[3])
                      : 0.0f,
      2);
  setDiagnosticValueUnsigned(4, "sequence", wheelCommandStatus.sequence);
  setDiagnosticValueBool(5, "commandFresh",
                         wheelCommandStatus.commandFresh);
#else
  status.level = static_cast<uint8_t>(
      diagnostic_msgs__msg__DiagnosticStatus__OK);
  setLiteralString(status.name, WHEEL_COMMAND_DIAG_NAME);
  setLiteralString(status.message, "DISABLED");
  status.values.size = 0;
#endif
}

void fillWheelFreshnessDiagnostic(
    uint32_t nowMs, diagnostic_msgs__msg__DiagnosticStatus &status) {
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
  const uint32_t localAgeMs = wheelCommandStatus.hasCurrentSessionSample
                                  ? nowMs - wheelCommandStatus.lastAcceptedRxMs
                                  : 0U;
  uint64_t sourceAgeMs = 0ULL;
  if (wheelCommandStatus.sourceTimeTrusted &&
      wheelCommandStatus.sampleTimeUnixUs != 0ULL) {
    const uint64_t nowUs = wheelEpochNowUs();
    if (nowUs >= wheelCommandStatus.sampleTimeUnixUs) {
      sourceAgeMs =
          (nowUs - wheelCommandStatus.sampleTimeUnixUs) / 1000ULL;
    }
  }
  status.level = wheelCommandStatus.commandFresh
                     ? static_cast<uint8_t>(
                           diagnostic_msgs__msg__DiagnosticStatus__OK)
                     : static_cast<uint8_t>(
                           diagnostic_msgs__msg__DiagnosticStatus__WARN);
  setLiteralString(status.name, WHEEL_FRESHNESS_DIAG_NAME);
  setLiteralString(status.message,
                   wheelRejectReasonText(wheelCommandStatus.lastRejectReason));
  status.values.size = DIAG_VALUE_CAPACITY;
  setDiagnosticValueUnsigned(0, "localAgeMs", localAgeMs);
  setDiagnosticValueUnsigned64(1, "sourceAgeMs", sourceAgeMs);
  setDiagnosticValueBool(2, "localFresh", wheelCommandStatus.localFresh);
  setDiagnosticValueBool(3, "sourceTrusted",
                         wheelCommandStatus.sourceTimeTrusted);
  setDiagnosticValueUnsigned(4, "rejectCount",
                             wheelCommandStatus.rejectCount);
  setDiagnosticValueUnsigned(5, "acceptedCount",
                             wheelCommandStatus.acceptedCount);
#else
  (void)nowMs;
  status.level = static_cast<uint8_t>(
      diagnostic_msgs__msg__DiagnosticStatus__OK);
  setLiteralString(status.name, WHEEL_FRESHNESS_DIAG_NAME);
  setLiteralString(status.message, "DISABLED");
  status.values.size = 0;
#endif
}

void updateOledRate(uint32_t nowMs) {
  ++oledUpdateCount;
  ++oledRateWindowUpdateCount;
  if (oledRateWindowStartMs == 0) {
    oledRateWindowStartMs = nowMs;
    return;
  }

  const uint32_t elapsedMs = nowMs - oledRateWindowStartMs;
  if (elapsedMs >= OLED_RATE_WINDOW_MS) {
    oledWindowHz = oledRateWindowUpdateCount * 1000.0f / elapsedMs;
    oledRateWindowStartMs = nowMs;
    oledRateWindowUpdateCount = 0;
    oledWindowMaxUs = oledLastUpdateUs;
  }
}

#if PICO_B_ENABLE_OLED
void printOledLine(int16_t x, int16_t y, const char *text) {
  display.setCursor(x, y);
  display.print(text);
}

void drawBatteryOledPage() {
  char line[24];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  printOledLine(0, 0, "PICO B   B2B");

  if (batteryStatus.valid && isfinite(batteryStatus.filteredVoltage)) {
    boundedSnprintf(line, sizeof(line), "BAT %.2f V",
                    static_cast<double>(batteryStatus.filteredVoltage));
  } else {
    const int written = snprintf(line, sizeof(line), "BAT --.-- V");
    if (written < 0 || static_cast<size_t>(written) >= sizeof(line)) {
      line[sizeof(line) - 1] = '\0';
    }
  }
  printOledLine(0, 16, line);

  if (batteryStatus.valid) {
    const int written = snprintf(line, sizeof(line), "SOC %u %%",
                                 static_cast<unsigned>(batteryStatus.socPercent));
    if (written < 0 || static_cast<size_t>(written) >= sizeof(line)) {
      line[sizeof(line) - 1] = '\0';
    }
  } else {
    const int written = snprintf(line, sizeof(line), "SOC --- %%");
    if (written < 0 || static_cast<size_t>(written) >= sizeof(line)) {
      line[sizeof(line) - 1] = '\0';
    }
  }
  printOledLine(0, 32, line);

  const int written = snprintf(line, sizeof(line), "STATE %s", batteryStateText());
  if (written < 0 || static_cast<size_t>(written) >= sizeof(line)) {
    line[sizeof(line) - 1] = '\0';
  }
  printOledLine(0, 48, line);
}

void drawSystemOledPage(uint32_t nowMs) {
  char line[24];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  const int sbusWritten = snprintf(line, sizeof(line), "SBUS %s", sbusStateText());
  if (sbusWritten < 0 || static_cast<size_t>(sbusWritten) >= sizeof(line)) {
    line[sizeof(line) - 1] = '\0';
  }
  printOledLine(0, 0, line);

  boundedSnprintf(line, sizeof(line), "HZ %.1f",
                  static_cast<double>(sbusStatus.frameRateHz));
  printOledLine(0, 16, line);

  const int ageWritten = snprintf(line, sizeof(line), "AGE %lu ms",
                                  static_cast<unsigned long>(sbusFrameAgeMs(nowMs)));
  if (ageWritten < 0 || static_cast<size_t>(ageWritten) >= sizeof(line)) {
    line[sizeof(line) - 1] = '\0';
  }
  printOledLine(0, 32, line);

  const char *rosText = "WAIT";
  if (rosState == RosState::RUNNING) {
    rosText = "RUN";
  } else if (rosState == RosState::DESTROY_ENTITIES) {
    rosText = "DESTROY";
  }
  const int rosWritten = snprintf(line, sizeof(line), "ROS %s", rosText);
  if (rosWritten < 0 || static_cast<size_t>(rosWritten) >= sizeof(line)) {
    line[sizeof(line) - 1] = '\0';
  }
  printOledLine(0, 48, line);
}

void drawMotorsOledPage(uint32_t nowMs) {
  char line[24];
  struct MotorOledSnapshot {
    const char *position;
    int16_t measuredRpm;
    uint8_t errorCode;
    bool online;
  };
  MotorOledSnapshot snapshot[MOTOR_COUNT];

  (void)nowMs;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const MotorStatus &status = motorStatus[i];
    snapshot[i].position = MOTOR_CONFIGS[i].position;
    snapshot[i].measuredRpm = status.measuredRpm;
    snapshot[i].errorCode = status.errorCode;
    snapshot[i].online = status.online;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  printOledLine(0, 0, "MOTORS");

  const int16_t y[MOTOR_COUNT] = {12, 24, 36, 48};
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const MotorOledSnapshot &status = snapshot[i];
    const char *stateLabel = "OFF";
    if (status.online && status.errorCode == 0x00) {
      stateLabel = "OK ";
    } else if (status.online && status.errorCode != 0x00) {
      stateLabel = "ERR";
    }
    const int written = snprintf(line, sizeof(line),
                                 "M%u %s %s %d",
                                 static_cast<unsigned>(i + 1),
                                 status.position,
                                 stateLabel,
                                 static_cast<int>(status.online
                                                      ? status.measuredRpm
                                                      : 0));
    if (written < 0 || static_cast<size_t>(written) >= sizeof(line)) {
      line[sizeof(line) - 1] = '\0';
    }
    printOledLine(0, y[i], line);
  }
}
#endif

void updateOled(uint32_t nowMs) {
#if PICO_B_ENABLE_OLED
  if (!oledAvailable) {
    return;
  }
  if (!motorAllowsSlowTasks()) {
    return;
  }
#if PICO_B_ENABLE_MANUAL_OLED_PAGING
  serviceOledManualPaging(nowMs);
#else
  if (periodElapsed(nowMs, lastOledPageChangeMs, OLED_PAGE_CHANGE_PERIOD_MS)) {
    oledPage = oledPage == OledPage::BATTERY ? OledPage::SYSTEM
                                             : OledPage::BATTERY;
  }
#endif
  if (!periodElapsed(nowMs, lastOledUpdateMs, OLED_UPDATE_PERIOD_MS)) {
    return;
  }

  sbusTask(nowMs);
  const uint32_t startUs = micros();
  switch (oledPage) {
    case OledPage::BATTERY:
      drawBatteryOledPage();
      break;
    case OledPage::SYSTEM:
      drawSystemOledPage(nowMs);
      break;
    case OledPage::MOTORS:
      drawMotorsOledPage(nowMs);
      break;
    default:
      drawSystemOledPage(nowMs);
      break;
  }
  display.display();
  oledLastUpdateUs = micros() - startUs;
  if (oledLastUpdateUs > oledWindowMaxUs) {
    oledWindowMaxUs = oledLastUpdateUs;
  }
  updateOledRate(nowMs);
  sbusTask(millis());
#else
  (void)nowMs;
#endif
}

void setupOled() {
#if PICO_B_ENABLE_OLED
  Wire.setSDA(OLED_SDA_PIN);
  Wire.setSCL(OLED_SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  oledAvailable = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (!oledAvailable) {
    ++oledInitFailureCount;
    return;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  printOledLine(0, 0, "PICO B   B2B");
  printOledLine(0, 16, "micro-ROS");
  printOledLine(0, 32, "OLED OK");
  display.display();
#endif
}

bool publishDiagnostics(uint32_t nowMs) {
#if PICO_B_ENABLE_ROS_DIAGNOSTICS
  rotateDiagnosticPage(nowMs);
  if (!periodElapsed(nowMs, lastDiagnosticPublishMs,
                     DIAGNOSTIC_PUBLISH_PERIOD_MS)) {
    return true;
  }

  diagnosticMsg.header.stamp.sec = 0;
  diagnosticMsg.header.stamp.nanosec = 0;
  diagnostic_msgs__msg__DiagnosticStatus &status = diagnosticStatusStorage[0];
  switch (diagnosticPage) {
    case DiagnosticPage::SBUS:
      fillSbusDiagnostic(nowMs, status);
      break;
    case DiagnosticPage::BATTERY:
      fillBatteryDiagnostic(status);
      break;
    case DiagnosticPage::MICRO_ROS:
      fillMicroRosDiagnostic(status);
      break;
    case DiagnosticPage::OLED:
      fillOledDiagnostic(status);
      break;
    case DiagnosticPage::MOTORS:
      fillMotorsDiagnostic(nowMs, status);
      break;
    case DiagnosticPage::MOTOR_1_FL:
      fillSingleMotorDiagnostic(0, nowMs, status);
      break;
    case DiagnosticPage::MOTOR_2_RL:
      fillSingleMotorDiagnostic(1, nowMs, status);
      break;
    case DiagnosticPage::MOTOR_3_FR:
      fillSingleMotorDiagnostic(2, nowMs, status);
      break;
    case DiagnosticPage::MOTOR_4_RR:
      fillSingleMotorDiagnostic(3, nowMs, status);
      break;
    case DiagnosticPage::MOTOR_COMM_12:
      fillMotorCommDiagnostic(0, status);
      break;
    case DiagnosticPage::MOTOR_COMM_34:
      fillMotorCommDiagnostic(2, status);
      break;
    case DiagnosticPage::MOTOR_DETAIL_1:
      fillMotorDetailDiagnostic(0, status);
      break;
    case DiagnosticPage::MOTOR_DETAIL_2:
      fillMotorDetailDiagnostic(1, status);
      break;
    case DiagnosticPage::MOTOR_DETAIL_3:
      fillMotorDetailDiagnostic(2, status);
      break;
    case DiagnosticPage::MOTOR_DETAIL_4:
      fillMotorDetailDiagnostic(3, status);
      break;
    case DiagnosticPage::MOTOR_LATENCY_12:
      fillMotorLatencyDiagnostic(0, status);
      break;
    case DiagnosticPage::MOTOR_LATENCY_34:
      fillMotorLatencyDiagnostic(2, status);
      break;
    case DiagnosticPage::MOTOR_TEMPERATURE_SCHEDULE:
      fillMotorTemperatureScheduleDiagnostic(status);
      break;
    case DiagnosticPage::MOTOR_SCHEDULER:
      fillMotorSchedulerDiagnostic(status);
      break;
    case DiagnosticPage::MOTOR_TRANSACTION_GUARD:
      fillMotorTransactionGuardDiagnostic(status);
      break;
    case DiagnosticPage::OLED_CONTROL:
      fillOledControlDiagnostic(nowMs, status);
      break;
    case DiagnosticPage::RELAYS:
      fillRelaysDiagnostic(status);
      break;
    case DiagnosticPage::CAMERA_SERVO:
      fillCameraServoDiagnostic(status);
      break;
    case DiagnosticPage::CMD_VEL:
      fillCmdVelDiagnostic(nowMs, status);
      break;
    case DiagnosticPage::WHEEL_COMMAND:
      fillWheelCommandDiagnostic(status);
      break;
    case DiagnosticPage::WHEEL_FRESHNESS:
      fillWheelFreshnessDiagnostic(nowMs, status);
      break;
  }

  const rcl_ret_t ret = rcl_publish(&diagnosticPublisher, &diagnosticMsg, nullptr);
  if (ret != RCL_RET_OK) {
    ++diagnosticPublishError;
    return false;
  }
#else
  (void)nowMs;
#endif
  return true;
}

bool checkRunningAgent(uint32_t nowMs) {
  if (static_cast<uint32_t>(nowMs - lastRunningAgentPingMs) <
      RUNNING_AGENT_PING_PERIOD_MS) {
    return true;
  }

  lastRunningAgentPingMs = nowMs;
  if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
    runningAgentPingFailures = 0;
    return true;
  }

  ++runningAgentPingFailures;
  return runningAgentPingFailures < RUNNING_AGENT_PING_FAILURE_LIMIT;
}

void telemetryTask(uint32_t nowMs) {
  (void)publishHeartbeat(nowMs);
  (void)publishSbus(nowMs);
  (void)publishBattery(nowMs);
  (void)publishMotorsRpm(nowMs);
  (void)publishDiagnostics(nowMs);
}

void rosTask(uint32_t nowMs) {
  switch (rosState) {
    case RosState::WAIT_AGENT:
      if (static_cast<uint32_t>(nowMs - lastAgentPingMs) <
          AGENT_PING_PERIOD_MS) {
        return;
      }
      lastAgentPingMs = nowMs;
      if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
        rosState = RosState::CREATE_ENTITIES;
      }
      return;

    case RosState::CREATE_ENTITIES:
      if (createEntities()) {
        rosState = RosState::RUNNING;
      } else {
        destroyEntities();
        ++reconnectCount;
        rosState = RosState::WAIT_AGENT;
      }
      return;

    case RosState::RUNNING:
      rosSessionError = false;
      if (!checkRunningAgent(nowMs)) {
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
        invalidateWheelSnapshot(WheelRejectReason::LIFECYCLE_RESET, true);
#endif
        rosState = RosState::DESTROY_ENTITIES;
        return;
      }
      telemetryTask(nowMs);
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
      {
        const rcl_ret_t execRet = rclc_executor_spin_some(&cmdVelExecutor, 0);
        (void)execRet;
      }
#endif
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
      {
        const rcl_ret_t execRet =
            rclc_executor_spin_some(&wheelCommandExecutor, 0);
        if (execRet != RCL_RET_OK && execRet != RCL_RET_TIMEOUT) {
          rejectWheelCommand(WheelRejectReason::EXECUTOR_ERROR);
          rosSessionError = true;
        }
      }
#endif
      if (rosSessionError) {
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
        invalidateWheelSnapshot(WheelRejectReason::LIFECYCLE_RESET, true);
#endif
        rosState = RosState::DESTROY_ENTITIES;
      }
      return;

    case RosState::DESTROY_ENTITIES:
      destroyEntities();
      ++reconnectCount;
      rosState = RosState::WAIT_AGENT;
      return;
  }
}

void setupPeripherals() {
#if PICO_B_ENABLE_BATTERY_ADC
  analogReadResolution(12);
  pinMode(BATTERY_ADC_PIN, INPUT);
#endif

#if PICO_B_ENABLE_SBUS
  setSbusSafeState();
  Serial2.setTX(SBUS_TX_PIN);
  Serial2.setRX(SBUS_RX_PIN);
  Serial2.setInvertRX(SBUS_RX_INVERTED);
  Serial2.setFIFOSize(128);
  Serial2.begin(SBUS_BAUD_RATE, SERIAL_8E2);
#else
  setSbusSafeState();
#endif

#if PICO_B_ENABLE_MOTORS
  Serial1.setTX(RS485_BRIDGE_TX_PIN);
  Serial1.setRX(RS485_BRIDGE_RX_PIN);
  Serial1.begin(RS485_BRIDGE_BAUD_RATE, SERIAL_8N1);
#endif
}

void setup() {
  digitalWrite(RELAY1_PIN, RELAY_INACTIVE_LEVEL);
  digitalWrite(RELAY2_PIN, RELAY_INACTIVE_LEVEL);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  setupPeripherals();
  setupOled();
  initSbusMessage();
  initMotorsRpmMessage();
  initDiagnosticMessage();
  set_microros_transports();
  zeroEntities();
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
  invalidateCmdVelSnapshot();
#endif
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
  invalidateWheelSnapshot(WheelRejectReason::LIFECYCLE_RESET, true);
#endif

  const uint32_t nowMs = millis();
  lastBatteryMeasurementMs = nowMs;
  initializeRelayControl(nowMs);
  initializeCameraServo(nowMs);
  initializeMotorTask(nowMs);
  measureBattery(nowMs);
}

void loop() {
  uint32_t nowMs = millis();
#if PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER
  wheelCommandStatus.sourceTimeMapped = rmw_uros_epoch_synchronized();
  wheelCommandStatus.sourceTimeTrusted =
      wheelCommandStatus.sourceTimeMapped &&
      (PICO_B_SOURCE_TIME_MAPPING_VERIFIED == 1);
  wheelEpochNowUsCached =
      wheelCommandStatus.sourceTimeTrusted ? wheelEpochNowUs() : 0ULL;
  if (wheelCommandStatus.hasCurrentSessionSample &&
      wheelCommandStatus.localFresh &&
      static_cast<uint32_t>(nowMs - wheelCommandStatus.lastAcceptedRxMs) >=
          WHEEL_COMMAND_FRESH_TIMEOUT_MS) {
    invalidateWheelSnapshot(WheelRejectReason::LOCAL_TIMEOUT, false);
  }
  if (wheelCommandStatus.hasCurrentSessionSample &&
      wheelCommandStatus.sourceTimeTrusted) {
    if (!rmw_uros_epoch_synchronized()) {
      invalidateWheelSnapshot(WheelRejectReason::TIME_UNSYNCED, false);
    } else {
      const uint64_t nowUs = wheelEpochNowUs();
      const uint64_t futureToleranceUs =
          static_cast<uint64_t>(WHEEL_COMMAND_SOURCE_FUTURE_TOLERANCE_MS) *
          1000ULL;
      if (nowUs == 0ULL ||
          wheelCommandStatus.sampleTimeUnixUs > nowUs + futureToleranceUs) {
        invalidateWheelSnapshot(WheelRejectReason::SOURCE_IN_FUTURE, false);
      } else if (nowUs >= wheelCommandStatus.sampleTimeUnixUs &&
                 nowUs - wheelCommandStatus.sampleTimeUnixUs >=
                     static_cast<uint64_t>(
                         WHEEL_COMMAND_FRESH_TIMEOUT_MS) *
                         1000ULL) {
        invalidateWheelSnapshot(WheelRejectReason::SOURCE_TIMEOUT, false);
      }
    }
  }
#endif
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
  if (cmdVelStatus.receiveCount > 0 && cmdVelStatus.commandFresh) {
    const uint32_t age = nowMs - cmdVelStatus.lastReceiveMs;
    if (age > CMD_VEL_FRESH_TIMEOUT_MS) {
      cmdVelStatus.commandFresh = false;
    }
  }
#endif
  sbusTask(nowMs);
  relayTask(nowMs);
  motorTask(nowMs);
  if (periodElapsed(nowMs, lastBatteryMeasurementMs,
                    BATTERY_MEASUREMENT_PERIOD_MS)) {
    measureBattery(nowMs);
  }
  nowMs = millis();
  motorTask(nowMs);
  relayTask(nowMs);
  if (motorAllowsSlowTasks()) {
    cameraServoTask(nowMs);
  }
  updateOled(nowMs);
  rosTask(millis());
}
