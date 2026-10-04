#include <Arduino.h>
#include <Wire.h>

#include <atomic>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <pico/time.h>
#include <hardware/watchdog.h>

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/ping.h>
#include <rmw_microros/time_sync.h>

#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int32.h>

#include "SparkFun_BNO08x_Arduino_Library.h"

struct uxrCustomTransport;

#ifndef PICO_A_ENABLE_IMU
#define PICO_A_ENABLE_IMU 1
#endif

#ifndef PICO_A_ENABLE_ULTRASONICS
#define PICO_A_ENABLE_ULTRASONICS 0
#endif

#ifndef PICO_A_ENABLE_GYRO_REPORT
#define PICO_A_ENABLE_GYRO_REPORT PICO_A_ENABLE_IMU
#endif

#ifndef PICO_A_ENABLE_ROS_IMU_PUBLISHER
#define PICO_A_ENABLE_ROS_IMU_PUBLISHER PICO_A_ENABLE_IMU
#endif

#ifndef PICO_A_ENABLE_ROS_RANGE_PUBLISHERS
#define PICO_A_ENABLE_ROS_RANGE_PUBLISHERS PICO_A_ENABLE_ULTRASONICS
#endif

#ifndef PICO_A_ENABLE_ROS_DIAGNOSTICS
#define PICO_A_ENABLE_ROS_DIAGNOSTICS 1
#endif
#ifndef PICO_A_ROS_DOMAIN_ID
#define PICO_A_ROS_DOMAIN_ID 42
#endif
#if PICO_A_ROS_DOMAIN_ID < 0 || PICO_A_ROS_DOMAIN_ID > 232
#error "PICO_A_ROS_DOMAIN_ID must be in valid DDS domain range [0..232]"
#endif

constexpr bool ENABLE_IMU = (PICO_A_ENABLE_IMU != 0);
constexpr bool ENABLE_ULTRASONICS = (PICO_A_ENABLE_ULTRASONICS != 0);
constexpr bool ENABLE_GYRO_REPORT = (PICO_A_ENABLE_GYRO_REPORT != 0);
constexpr bool ENABLE_ROS_IMU_PUBLISHER =
    (PICO_A_ENABLE_ROS_IMU_PUBLISHER != 0);
constexpr bool ENABLE_ROS_RANGE_PUBLISHERS =
    (PICO_A_ENABLE_ROS_RANGE_PUBLISHERS != 0);
constexpr bool ENABLE_ROS_DIAGNOSTICS = (PICO_A_ENABLE_ROS_DIAGNOSTICS != 0);

bool core1_separate_stack = true;

constexpr uint8_t BNO08X_SDA_PIN = 20;
constexpr uint8_t BNO08X_SCL_PIN = 21;
constexpr uint8_t BNO08X_INT_PIN = 12;
constexpr uint8_t BNO08X_RST_PIN = 13;
constexpr uint8_t BNO08X_I2C_ADDRESS = 0x4B;
constexpr uint32_t BNO08X_I2C_CLOCK_HZ = 100000;
constexpr uint32_t IMU_POLL_INTERVAL_US = 8000;
constexpr uint16_t IMU_REPORT_INTERVAL_MS = 20;
constexpr uint32_t IMU_REPORT_RETRY_US = 1000000;
constexpr uint32_t IMU_REINIT_BACKOFF_US = 2000000;
constexpr uint32_t IMU_RATE_WINDOW_US = 1000000;
constexpr uint32_t IMU_FRESH_US = 50000;
constexpr uint32_t IMU_STALE_US = 200000;
constexpr uint32_t IMU_REINIT_AGE_US = 1000000;
constexpr uint32_t IMU_PUBLISH_MAX_AGE_US = 40000;
constexpr uint32_t IMU_PAIR_MAX_SKEW_US = 25000;
constexpr uint8_t BNO_EVENT_DRAIN_LIMIT = 4;
constexpr uint32_t BNO_EVENT_DRAIN_BUDGET_US = 12000;
constexpr float IMU_QUAT_FILTER_CUTOFF_HZ = 10.0F;
constexpr float IMU_MIN_QUATERNION_NORM = 0.80F;
constexpr float IMU_MAX_QUATERNION_NORM = 1.20F;
constexpr float IMU_PI_F = 3.14159265358979323846F;
constexpr double IMU_ORIENTATION_VARIANCE = 0.00030461741978670857;
constexpr double IMU_GYRO_VARIANCE = 0.0004;
constexpr float GYRO_MAX_ABS_RAD_PER_SEC = 34.906585F;

constexpr uint8_t SENSOR_COUNT = 5;
constexpr uint32_t TRIGGER_LOW_US = 2;
constexpr uint32_t TRIGGER_HIGH_US = 10;
constexpr uint32_t ECHO_TIMEOUT_US = 15000;
constexpr uint32_t QUIET_GAP_US = 3000;
constexpr uint32_t SENSOR_SLOT_US = ECHO_TIMEOUT_US + QUIET_GAP_US;
constexpr float SOUND_CM_PER_US = 0.0343F;
constexpr float MIN_DISTANCE_CM = 2.0F;
constexpr float MAX_DISTANCE_CM = 250.0F;
constexpr float ULTRASONIC_EMA_ALPHA = 0.35F;
constexpr float MEDIAN_REJECT_DIAGNOSTIC_CM = 50.0F;
constexpr uint64_t HOLD_MAX_AGE_US = 300000;
constexpr uint64_t ULTRASONIC_RATE_WINDOW_US = 1000000;
constexpr int8_t NO_ACTIVE_SENSOR = -1;

constexpr uint32_t HEARTBEAT_PERIOD_MS = 500;
constexpr uint32_t DIAGNOSTIC_PERIOD_MS = 500;
constexpr uint32_t OBSTACLE_PERIOD_MS = 100;
constexpr uint32_t RANGE_PUBLISH_MIN_PERIOD_US = 85000;
constexpr uint32_t AGENT_PING_PERIOD_MS = 500;
constexpr int AGENT_PING_TIMEOUT_MS = 20;
constexpr int TIME_SYNC_TIMEOUT_MS = 100;
constexpr uint32_t RUNNING_AGENT_PING_PERIOD_MS = 1000;
constexpr uint8_t  RUNNING_AGENT_PING_FAILURE_LIMIT = 2;
constexpr uint8_t LED_PIN = LED_BUILTIN;
constexpr uint32_t BOOT_MARKER_MAGIC = 0x5032534EU;

constexpr uint8_t SENSOR_PINS[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
constexpr uint8_t RESERVED_PINS[] = {12, 13, 16, 17, 18, 19, 20, 21};
constexpr float OBSTACLE_STOP_CM[SENSOR_COUNT] = {30.0F, 20.0F, 25.0F,
                                                   25.0F, 20.0F};

constexpr bool ultrasonicPinsAreSafe() {
  for (size_t i = 0; i < sizeof(SENSOR_PINS); ++i) {
    for (size_t j = i + 1; j < sizeof(SENSOR_PINS); ++j) {
      if (SENSOR_PINS[i] == SENSOR_PINS[j]) {
        return false;
      }
    }
    for (size_t j = 0; j < sizeof(RESERVED_PINS); ++j) {
      if (SENSOR_PINS[i] == RESERVED_PINS[j]) {
        return false;
      }
    }
  }
  return true;
}

static_assert(IMU_POLL_INTERVAL_US == 8000,
              "Validated BNO08X poll interval must remain 8 ms");
static_assert(BNO08X_I2C_CLOCK_HZ == 100000,
              "Golden IMU I2C clock must remain 100 kHz");
static_assert(ultrasonicPinsAreSafe(),
              "Ultrasonic pins must be unique and avoid IMU/SPI pins");
static_assert(ECHO_TIMEOUT_US >= 14578,
              "Echo timeout must cover the configured 250 cm range");
static_assert(SENSOR_SLOT_US * SENSOR_COUNT == 90000,
              "Five sensor slots must target a 90 ms full scan");

enum class ImuRunState : uint8_t {
  DISABLED,
  OFFLINE,
  INITIALIZING,
  WAITING_FIRST_SAMPLE,
  RUNNING,
  STALE,
  BACKOFF
};

enum class ScanState : uint8_t {
  SELECT_SENSOR,
  TRIGGER_LOW,
  TRIGGER_HIGH,
  WAIT_ECHO_RISE,
  WAIT_ECHO_FALL,
  QUIET_GAP,
  NEXT_SENSOR
};

enum class EchoCapturePhase : uint8_t {
  IGNORE,
  WAIT_RISE,
  WAIT_FALL
};

enum class SensorHealth : uint8_t {
  DISABLED,
  INIT,
  VALID,
  HOLD,
  UNKNOWN
};

enum class RosState : uint8_t {
  WAIT_AGENT,
  CREATE_ENTITIES,
  SYNC_TIME,
  RUNNING,
  DESTROY_ENTITIES
};

enum class BootStage : uint32_t {
  START = 0,
  TRANSPORT_CONFIGURED = 1,
  SENSOR_SHARED_STATE_READY = 2,
  CORE1_RELEASED = 3,
  ALLOCATOR_SUPPORT_READY = 4,
  NODE_READY = 5,
  PUBLISHERS_READY = 6,
  TIME_SYNC_READY = 7,
  RUNNING = 8
};

struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

struct Vector3 {
  float x;
  float y;
  float z;
};

struct ImuSnapshot {
  Quaternion raw;
  Quaternion filtered;
  Vector3 gyro;
  float quaternion_norm;
  float measured_hz;
  float gyro_hz;
  float publish_hz;
  float last_dt_ms;
  uint64_t sample_time_us;
  uint64_t sensor_timestamp_us;
  uint64_t gyro_sample_time_us;
  uint64_t gyro_sensor_timestamp_us;
  uint32_t sequence;
  uint32_t gyro_sequence;
  uint32_t sample_age_us;
  uint32_t gyro_sample_age_us;
  uint32_t pair_skew_us;
  uint32_t pair_skew_max_us;
  uint32_t bno_events_per_service;
  uint32_t bno_event_drain_max;
  uint32_t sensor_task_max_us;
  uint32_t imu_service_last_us;
  uint32_t imu_service_window_max_us;
  uint32_t imu_service_overrun_count;
  uint32_t imu_poll_miss_count;
  uint32_t imu_queue_still_asserted_count;
  uint32_t sample_count;
  uint32_t gyro_sample_count;
  uint32_t pair_publish_count;
  uint32_t false_count;
  uint32_t invalid_count;
  uint32_t gyro_invalid_count;
  uint32_t nan_count;
  uint32_t gyro_nan_count;
  uint32_t norm_error_count;
  uint32_t timing_error_count;
  uint32_t reset_count;
  uint32_t report_error_count;
  uint32_t gyro_reenable_fail_count;
  uint32_t init_error_count;
  uint32_t stale_count;
  uint32_t gyro_stale_count;
  uint32_t pair_skew_reject_count;
  bool initialized;
  bool report_enabled;
  bool gyro_report_enabled;
  bool valid;
  bool gyro_valid;
  bool stale;
  bool gyro_stale;
  bool time_valid;
  uint8_t orientation_accuracy;
  uint8_t gyro_accuracy;
  ImuRunState state;
};

struct ImuState : ImuSnapshot {
  uint32_t last_valid_us;
  uint32_t previous_valid_us;
  uint32_t next_poll_us;
  uint32_t rate_window_start_us;
  uint32_t rate_window_samples;
  uint32_t rate_window_gyro_samples;
  uint32_t publish_window_start_us;
  uint32_t publish_window_samples;
  bool filter_initialized;
};

struct UltrasonicSensor {
  uint8_t id;
  const char *name;
  uint8_t trig_pin;
  uint8_t echo_pin;
  uint32_t pulse_width_us;
  uint64_t last_update_us;
  uint32_t valid_count;
  uint32_t timeout_count;
  uint32_t glitch_count;
  uint32_t range_error_count;
  float raw_cm;
  float median_cm;
  float filtered_cm;
  float safety_cm;
  float valid_history[3];
  uint8_t history_count;
  uint8_t history_index;
  uint32_t raw_sequence;
  uint32_t filter_sequence;
  uint64_t raw_sample_time_us;
  uint64_t filter_sample_time_us;
  uint32_t consecutive_timeout_count;
  uint32_t median_reject_count;
  uint32_t filter_reset_count;
  bool raw_valid;
  bool timeout;
  bool filter_initialized;
  bool safety_valid;
  SensorHealth health;
};

struct EchoCapture {
  volatile uint64_t rise_time_us;
  volatile uint64_t fall_time_us;
  volatile uint8_t last_level;
  volatile bool rise_ready;
  volatile bool fall_ready;
};

struct DiagnosticBuffers {
  diagnostic_msgs__msg__DiagnosticStatus statuses[3];
  diagnostic_msgs__msg__KeyValue values_kv[6];
  char values[6][56];
};

struct BootMarker {
  uint32_t magic;
  uint32_t boot_count;
  uint32_t previous_stage;
  uint32_t current_stage;
  uint32_t watchdog_reboot_count;
  uint32_t non_watchdog_reboot_count;
};

static_assert(sizeof(ImuSnapshot) % sizeof(uint32_t) == 0,
              "IMU snapshot is copied as 32-bit atomic words");

BNO08x imu;
ImuState imuState = {};
uint32_t nextInitAttemptUs = 0;
uint32_t nextReportAttemptUs = 0;
uint32_t firstSampleWaitStartUs = 0;
bool reportRetryPending = false;

constexpr size_t IMU_SNAPSHOT_WORDS = sizeof(ImuSnapshot) / sizeof(uint32_t);
std::atomic<uint32_t> imuSnapshotWords[IMU_SNAPSHOT_WORDS];
std::atomic<uint32_t> imuSnapshotVersion{0};
std::atomic<uint32_t> imuCoreReady{0};
std::atomic<uint32_t> core0ReadyForCore1{0};
volatile BootMarker bootMarker __attribute__((section(".noinit")));
uint32_t bootPreviousStage = 0;
uint32_t bootCount = 0;
bool bootWasWatchdog = false;
bool transportSerialStarted = false;

UltrasonicSensor sensors[SENSOR_COUNT] = {
    {1, "front", 2, 3}, {2, "top", 4, 5}, {3, "left", 6, 7},
    {4, "right", 8, 9}, {5, "rear", 10, 11}};
EchoCapture echoCapture[SENSOR_COUNT] = {};
volatile uint32_t isrGlitchPending[SENSOR_COUNT] = {};
volatile EchoCapturePhase echoCapturePhase = EchoCapturePhase::IGNORE;
volatile int8_t activeSensorForIsr = NO_ACTIVE_SENSOR;
ScanState scanState = ScanState::SELECT_SENSOR;
uint8_t activeSensorIndex = 0;
uint32_t stateStartUs = 0;
uint32_t sensorSlotStartUs = 0;
uint32_t echoWaitStartUs = 0;
uint64_t scanWindowStartUs = 0;
uint32_t scansInWindow = 0;
float scanHz = 0.0F;
uint32_t lastRangePublishSeq[SENSOR_COUNT] = {};
uint64_t lastRangePublishUs[SENSOR_COUNT] = {};

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t heartbeatPub;
rcl_publisher_t imuPub;
rcl_publisher_t rangePubs[SENSOR_COUNT];
rcl_publisher_t obstaclePub;
rcl_publisher_t diagnosticsPub;
std_msgs__msg__UInt32 heartbeatMsg;
sensor_msgs__msg__Imu imuMsg;
sensor_msgs__msg__Range rangeMsgs[SENSOR_COUNT];
std_msgs__msg__Bool obstacleMsg;
diagnostic_msgs__msg__DiagnosticArray diagnosticMsg;
DiagnosticBuffers diagnosticBuffers;

RosState rosState = RosState::WAIT_AGENT;
uint32_t lastHeartbeatMs = 0;
uint32_t lastDiagnosticMs = 0;
uint32_t lastObstacleMs = 0;
uint32_t lastAgentPingMs = 0;
uint32_t lastRunningAgentPingMs = 0;
uint32_t heartbeatCount = 0;
uint32_t publishErrorCount = 0;
uint32_t diagnosticPublishError = 0;
uint32_t reconnectCount = 0;
uint32_t timeSyncCount = 0;
uint32_t entityCreateErrorStep = 0;
uint32_t loopMaxUs = 0;
uint32_t triggerHighMaxUs = 0;
uint32_t imuPairSkewRejectCount = 0;
uint32_t imuPairPublishCount = 0;
uint32_t imuPairSkewUs = 0;
uint32_t imuPairSkewMaxUs = 0;
uint32_t rejectOrientationNotNew = 0;
uint32_t rejectGyroNotNew = 0;
uint32_t rejectFreshness = 0;
uint32_t rejectSkew = 0;
uint32_t rejectInvalid = 0;
uint32_t rejectRosTime = 0;
uint32_t lastRejectOrientationNotNewSequence = UINT32_MAX;
uint32_t lastRejectFreshnessSequence = UINT32_MAX;
uint32_t lastRejectSkewSequence = UINT32_MAX;
uint32_t lastRejectInvalidSequence = UINT32_MAX;
uint32_t lastRejectRosTimeSequence = UINT32_MAX;
bool rejectSnapshotReadCounted = false;
uint32_t imuPublishWindowStartUs = 0;
uint32_t imuPublishWindowSamples = 0;
float imuPublishHz = 0.0F;
uint8_t diagnosticPage = 0;
bool entitiesCreated = false;
bool timeSynchronized = false;
bool rosSessionError = false;
uint8_t runningAgentPingFailures = 0;
int64_t epochOffsetNs = 0;
int64_t lastRosStampNs = 0;

char frameImu[] = "imu_link";
char frameU1[] = "ultrasonic_front_link";
char frameU2[] = "ultrasonic_top_link";
char frameU3[] = "ultrasonic_left_link";
char frameU4[] = "ultrasonic_right_link";
char frameU5[] = "ultrasonic_rear_link";
char frameDiag[] = "pico_a";
char topicRange0[] = "/pico_a/range/front";
char topicRange1[] = "/pico_a/range/top";
char topicRange2[] = "/pico_a/range/left";
char topicRange3[] = "/pico_a/range/right";
char topicRange4[] = "/pico_a/range/rear";
char *rangeFrames[SENSOR_COUNT] = {frameU1, frameU2, frameU3, frameU4,
                                    frameU5};
const char *rangeTopics[SENSOR_COUNT] = {topicRange0, topicRange1, topicRange2,
                                         topicRange3, topicRange4};

char diagNameImu[] = "pico_a/imu";
char diagNameUltrasonic[] = "pico_a/ultrasonics";
char diagNameMicroRos[] = "pico_a/micro_ros";
char diagHardware[] = "pico_a_rp2350";
char diagMsgOk[] = "OK";
char diagMsgWarn[] = "WARN";
char diagMsgError[] = "ERROR";
char diagMsgDisabled[] = "DISABLED";
char diagMsgSensorUnknown[] = "SENSOR_UNKNOWN";

char keyState[] = "state";
char keyHz[] = "hz";
char keySampleAgeUs[] = "sample_age_us";
char keySourceHz[] = "source_hz";
char keySeq[] = "seq";
char keyNorm[] = "quat_norm";
char keySamples[] = "samples";
char keyFalse[] = "false";
char keyInvalid[] = "invalid";
char keyNan[] = "nan";
char keyNormErr[] = "norm_err";
char keyReinit[] = "reinit";
char keyStale[] = "stale";
char keyFlags[] = "flags";
char keyScanHz[] = "scan_hz";
char keyObstacle[] = "obstacle";
char keyTimeout[] = "timeout";
char keyGlitch[] = "glitch";
char keyRangeErr[] = "range_err";
char keyReject[] = "reject";
char keyReset[] = "filter_reset";
char keyTransport[] = "transport";
char keyHeartbeat[] = "heartbeat";
char keyPubErr[] = "pub_err";
char keyReconnect[] = "reconnect";
char keyTimeSync[] = "time_sync";
char keyLibrary[] = "library";
char keyImuOrientationRawHz[] = "imuOrientationRawHz";
char keyImuGyroRawHz[] = "imuGyroRawHz";
char keyImuPublishHz[] = "imuPublishHz";
char keyOrientationSequence[] = "orientationSequence";
char keyGyroSequence[] = "gyroSequence";
char keyOrientationAgeUs[] = "orientationAgeUs";
char keyGyroAgeUs[] = "gyroAgeUs";
char keyOrientationGyroSkewUs[] = "orientationGyroSkewUs";
char keyBnoEventsPerService[] = "bnoEventsPerService";
char keyBnoEventDrainMax[] = "bnoEventDrainMax";
char keyRejectOrientationNotNew[] = "rejectOrientationNotNew";
char keyRejectGyroNotNew[] = "rejectGyroNotNew";
char keyRejectFreshness[] = "rejectFreshness";
char keyRejectSkew[] = "rejectSkew";
char keyRejectInvalid[] = "rejectInvalid";
char keyRejectRosTime[] = "rejectRosTime";
char keyPublishError[] = "publishError";
char keySensorTaskMaxUs[] = "sensorTaskMaxUs";
char keyImuServiceLastUs[] = "imuServiceLastUs";
char keyImuServiceWindowMaxUs[] = "imuServiceWindowMaxUs";
char keyImuServiceOverrunCount[] = "imuServiceOverrunCount";
char keyImuPollMissCount[] = "imuPollMissCount";
char keyImuQueueStillAssertedCount[] = "imuQueueStillAssertedCount";
char keyOrientationAccuracy[] = "orientationAccuracy";
char keyAgentState[] = "agentState";
char keyTimeSynced[] = "timeSynced";
char keyPublishErrors[] = "publishErrors";
char keyDisconnects[] = "disconnects";
char keyReconnects[] = "reconnects";
char keyEntityCreateFailures[] = "entityCreateFailures";
char keyLoopMaxUs[] = "loopMaxUs";
char keyTriggerHighMaxUs[] = "triggerHighMaxUs";
char keyBootStage[] = "bootStage";
char keyWatchdogReset[] = "watchdogReset";
char keyPage[] = "page";
char diagPageImuRates[] = "imu_rates";
char diagPageImuTiming[] = "imu_timing";
char diagPageImuRejectA[] = "imu_reject_a";
char diagPageImuRejectB[] = "imu_reject_b";
char diagPageImuService[] = "imu_service";
char diagPageUltrasonicA[] = "ultrasonic_a";
char diagPageUltrasonicB[] = "ultrasonic_b";
char diagPageUltrasonicC[] = "ultrasonic_c";
char diagPageMicroRos[] = "micro_ros";
char ultraKeyState[5][9] = {"U1_state", "U2_state", "U3_state", "U4_state",
                            "U5_state"};
char ultraKeyRange[5][9] = {"U1_range", "U2_range", "U3_range", "U4_range",
                            "U5_range"};
char ultraKeyAge[5][7] = {"U1_age", "U2_age", "U3_age", "U4_age", "U5_age"};
char ultraKeyTimeout[5][11] = {"U1_timeout", "U2_timeout", "U3_timeout",
                               "U4_timeout", "U5_timeout"};
char ultraKeyGlitch[5][10] = {"U1_glitch", "U2_glitch", "U3_glitch",
                              "U4_glitch", "U5_glitch"};
char ultraKeyRangeError[5][14] = {"U1_rangeError", "U2_rangeError",
                                  "U3_rangeError", "U4_rangeError",
                                  "U5_rangeError"};

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

void setBootStage(BootStage stage) {
  bootMarker.magic = BOOT_MARKER_MAGIC;
  bootMarker.current_stage = static_cast<uint32_t>(stage);
}

void initializeBootMarker() {
  const bool valid = bootMarker.magic == BOOT_MARKER_MAGIC;
  const uint32_t previousBootCount = valid ? bootMarker.boot_count : 0U;
  const uint32_t previousWatchdogCount =
      valid ? bootMarker.watchdog_reboot_count : 0U;
  const uint32_t previousNonWatchdogCount =
      valid ? bootMarker.non_watchdog_reboot_count : 0U;
  bootPreviousStage = valid ? bootMarker.current_stage : 0U;
  bootWasWatchdog = watchdog_caused_reboot();
  bootCount = previousBootCount + 1U;

  bootMarker.magic = BOOT_MARKER_MAGIC;
  bootMarker.boot_count = bootCount;
  bootMarker.previous_stage = bootPreviousStage;
  bootMarker.current_stage = static_cast<uint32_t>(BootStage::START);
  bootMarker.watchdog_reboot_count =
      previousWatchdogCount + (bootWasWatchdog ? 1U : 0U);
  bootMarker.non_watchdog_reboot_count =
      previousNonWatchdogCount + (bootWasWatchdog ? 0U : 1U);
}

void blinkPreviousBootStage() {
  if (bootPreviousStage == 0U || bootPreviousStage > 8U) {
    return;
  }
  for (uint32_t i = 0; i < bootPreviousStage; ++i) {
    digitalWrite(LED_PIN, HIGH);
    delay(80);
    digitalWrite(LED_PIN, LOW);
    delay(120);
  }
  delay(300);
}

bool timeReachedUs(uint32_t nowUs, uint32_t targetUs) {
  return static_cast<int32_t>(nowUs - targetUs) >= 0;
}

bool elapsedUs(uint32_t nowUs, uint32_t startUs, uint32_t intervalUs) {
  return static_cast<uint32_t>(nowUs - startUs) >= intervalUs;
}

void countUniqueOrientationReject(uint32_t &counter, uint32_t &lastSequence,
                                  uint32_t sequence) {
  if (sequence == lastSequence) {
    return;
  }
  ++counter;
  lastSequence = sequence;
}

void advanceImuPollDeadline(uint32_t nowUs) {
  uint32_t nextPollUs = imuState.next_poll_us + IMU_POLL_INTERVAL_US;
  uint32_t skippedDeadlines = 0;
  while (timeReachedUs(nowUs, nextPollUs) && skippedDeadlines < 8U) {
    nextPollUs += IMU_POLL_INTERVAL_US;
    ++skippedDeadlines;
  }
  if (timeReachedUs(nowUs, nextPollUs)) {
    nextPollUs = nowUs + IMU_POLL_INTERVAL_US;
    ++skippedDeadlines;
  }
  if (skippedDeadlines > 0U) {
    imuState.imu_poll_miss_count += skippedDeadlines;
  }
  imuState.next_poll_us = nextPollUs;
}

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

float quaternionNorm(const Quaternion &quaternion) {
  return sqrtf(quaternion.w * quaternion.w + quaternion.x * quaternion.x +
               quaternion.y * quaternion.y + quaternion.z * quaternion.z);
}

float quaternionDot(const Quaternion &first, const Quaternion &second) {
  return first.w * second.w + first.x * second.x + first.y * second.y +
         first.z * second.z;
}

bool normalizeQuaternionUnchecked(const Quaternion &input,
                                  Quaternion &normalized) {
  const float norm = quaternionNorm(input);
  if (!isfinite(norm) || norm <= 0.000001F) {
    return false;
  }
  const float inverseNorm = 1.0F / norm;
  normalized = {input.w * inverseNorm, input.x * inverseNorm,
                input.y * inverseNorm, input.z * inverseNorm};
  return true;
}

const char *imuStateName(ImuRunState state) {
  switch (state) {
  case ImuRunState::DISABLED:
    return "DISABLED";
  case ImuRunState::OFFLINE:
    return "OFFLINE";
  case ImuRunState::INITIALIZING:
    return "INIT";
  case ImuRunState::WAITING_FIRST_SAMPLE:
    return "WAIT";
  case ImuRunState::RUNNING:
    return "RUN";
  case ImuRunState::STALE:
    return "STALE";
  case ImuRunState::BACKOFF:
    return "BACKOFF";
  }
  return "UNKNOWN";
}

const char *healthName(SensorHealth health) {
  switch (health) {
  case SensorHealth::DISABLED:
    return "DISABLED";
  case SensorHealth::INIT:
    return "INIT";
  case SensorHealth::VALID:
    return "VALID";
  case SensorHealth::HOLD:
    return "HOLD";
  case SensorHealth::UNKNOWN:
    return "UNKNOWN";
  }
  return "UNKNOWN";
}

const char *sensorHealthName(SensorHealth health) {
  switch (health) {
  case SensorHealth::DISABLED:
    return "DISABLED";
  case SensorHealth::INIT:
    return "INIT";
  case SensorHealth::VALID:
    return "VALID";
  case SensorHealth::HOLD:
    return "HOLD";
  case SensorHealth::UNKNOWN:
    return "UNKNOWN";
  }
  return "UNKNOWN";
}

uint32_t saturatingAgeUs(uint64_t nowUs, uint64_t sampleUs) {
  if (sampleUs == 0U) {
    return UINT32_MAX;
  }
  if (sampleUs > nowUs) {
    return 0U;
  }
  const uint64_t ageUs = nowUs - sampleUs;
  return ageUs > static_cast<uint64_t>(UINT32_MAX)
             ? UINT32_MAX
             : static_cast<uint32_t>(ageUs);
}

float clampDistance(float distanceCm) {
  return clampFloat(distanceCm, MIN_DISTANCE_CM, MAX_DISTANCE_CM);
}

float medianOfThree(float first, float second, float third) {
  if ((first <= second && second <= third) ||
      (third <= second && second <= first)) {
    return second;
  }
  if ((second <= first && first <= third) ||
      (third <= first && first <= second)) {
    return first;
  }
  return third;
}

Quaternion slerpQuaternion(const Quaternion &from, const Quaternion &to,
                           float alpha) {
  Quaternion target = to;
  float dot = quaternionDot(from, target);
  if (dot < 0.0F) {
    target = {-target.w, -target.x, -target.y, -target.z};
    dot = -dot;
  }
  dot = clampFloat(dot, -1.0F, 1.0F);
  const float clampedAlpha = clampFloat(alpha, 0.0F, 1.0F);
  Quaternion result;
  if (dot > 0.9995F) {
    result = {from.w + clampedAlpha * (target.w - from.w),
              from.x + clampedAlpha * (target.x - from.x),
              from.y + clampedAlpha * (target.y - from.y),
              from.z + clampedAlpha * (target.z - from.z)};
  } else {
    const float theta = acosf(dot);
    const float sinTheta = sinf(theta);
    if (fabsf(sinTheta) <= 0.000001F) {
      result = target;
    } else {
      const float fromWeight = sinf((1.0F - clampedAlpha) * theta) / sinTheta;
      const float toWeight = sinf(clampedAlpha * theta) / sinTheta;
      result = {fromWeight * from.w + toWeight * target.w,
                fromWeight * from.x + toWeight * target.x,
                fromWeight * from.y + toWeight * target.y,
                fromWeight * from.z + toWeight * target.z};
    }
  }
  Quaternion normalized = target;
  (void)normalizeQuaternionUnchecked(result, normalized);
  return normalized;
}

bool validateAndNormalizeQuaternion(const Quaternion &input,
                                    const Quaternion *previous,
                                    Quaternion &normalized) {
  if (!isfinite(input.w) || !isfinite(input.x) || !isfinite(input.y) ||
      !isfinite(input.z)) {
    ++imuState.nan_count;
    ++imuState.invalid_count;
    return false;
  }
  const float norm = quaternionNorm(input);
  if (!isfinite(norm)) {
    ++imuState.nan_count;
    ++imuState.invalid_count;
    return false;
  }
  if (norm < IMU_MIN_QUATERNION_NORM ||
      norm > IMU_MAX_QUATERNION_NORM) {
    ++imuState.norm_error_count;
    ++imuState.invalid_count;
    return false;
  }
  const float inverseNorm = 1.0F / norm;
  normalized = {input.w * inverseNorm, input.x * inverseNorm,
                input.y * inverseNorm, input.z * inverseNorm};
  if (previous != nullptr && quaternionDot(*previous, normalized) < 0.0F) {
    normalized.w = -normalized.w;
    normalized.x = -normalized.x;
    normalized.y = -normalized.y;
    normalized.z = -normalized.z;
  }
  return true;
}

uint32_t refreshImuSampleAgeUs() {
  if (imuState.sample_time_us == 0U) {
    imuState.sample_age_us = UINT32_MAX;
  } else {
    imuState.sample_age_us =
        saturatingAgeUs(time_us_64(), imuState.sample_time_us);
  }
  if (imuState.gyro_sample_time_us == 0U) {
    imuState.gyro_sample_age_us = UINT32_MAX;
  } else {
    imuState.gyro_sample_age_us =
        saturatingAgeUs(time_us_64(), imuState.gyro_sample_time_us);
  }
  return imuState.sample_age_us;
}

void publishImuSnapshot() {
  ImuSnapshot snapshot;
  memcpy(&snapshot, &imuState, sizeof(snapshot));
  const uint32_t *words = reinterpret_cast<const uint32_t *>(&snapshot);
  const uint32_t startVersion =
      imuSnapshotVersion.load(std::memory_order_relaxed) + 1U;
  imuSnapshotVersion.store(startVersion, std::memory_order_release);
  for (size_t i = 0; i < IMU_SNAPSHOT_WORDS; ++i) {
    imuSnapshotWords[i].store(words[i], std::memory_order_relaxed);
  }
  imuSnapshotVersion.store(startVersion + 1U, std::memory_order_release);
}

bool readImuSnapshot(ImuSnapshot &snapshot) {
  uint32_t *words = reinterpret_cast<uint32_t *>(&snapshot);
  for (uint8_t attempt = 0; attempt < 3U; ++attempt) {
    const uint32_t before = imuSnapshotVersion.load(std::memory_order_acquire);
    if ((before & 1U) != 0U) {
      continue;
    }
    for (size_t i = 0; i < IMU_SNAPSHOT_WORDS; ++i) {
      words[i] = imuSnapshotWords[i].load(std::memory_order_relaxed);
    }
    const uint32_t after = imuSnapshotVersion.load(std::memory_order_acquire);
    if (before == after && (after & 1U) == 0U) {
      return true;
    }
  }
  return false;
}

void scheduleInitializationBackoff(uint32_t nowUs) {
  imuState.initialized = false;
  imuState.report_enabled = false;
  imuState.gyro_report_enabled = false;
  imuState.filter_initialized = false;
  imuState.valid = false;
  imuState.gyro_valid = false;
  imuState.stale = true;
  imuState.gyro_stale = true;
  imuState.time_valid = false;
  imuState.state = ImuRunState::BACKOFF;
  reportRetryPending = false;
  nextInitAttemptUs = nowUs + IMU_REINIT_BACKOFF_US;
  publishImuSnapshot();
}

bool enableImuReports() {
  const bool gameEnabled = imu.enableGameRotationVector(IMU_REPORT_INTERVAL_MS);
  const bool gyroEnabled =
      !ENABLE_GYRO_REPORT || imu.enableGyro(IMU_REPORT_INTERVAL_MS);
  const uint32_t completedUs = micros();
  imuState.report_enabled = gameEnabled;
  imuState.gyro_report_enabled = gyroEnabled;
  if (!gameEnabled || !gyroEnabled) {
    ++imuState.report_error_count;
    if (!gyroEnabled) {
      ++imuState.gyro_reenable_fail_count;
    }
    imuState.valid = false;
    imuState.gyro_valid = false;
    imuState.stale = true;
    imuState.gyro_stale = true;
    imuState.time_valid = false;
    imuState.state = ImuRunState::BACKOFF;
    reportRetryPending = true;
    nextReportAttemptUs = completedUs + IMU_REPORT_RETRY_US;
    publishImuSnapshot();
    return false;
  }
  reportRetryPending = false;
  imuState.filter_initialized = false;
  imuState.valid = false;
  imuState.gyro_valid = !ENABLE_GYRO_REPORT;
  imuState.stale = false;
  imuState.gyro_stale = ENABLE_GYRO_REPORT;
  imuState.time_valid = false;
  imuState.state = ImuRunState::WAITING_FIRST_SAMPLE;
  imuState.next_poll_us = completedUs + IMU_POLL_INTERVAL_US;
  imuState.rate_window_start_us = completedUs;
  imuState.rate_window_samples = 0;
  imuState.rate_window_gyro_samples = 0;
  imuState.publish_window_start_us = completedUs;
  imuState.publish_window_samples = 0;
  firstSampleWaitStartUs = completedUs;
  publishImuSnapshot();
  return true;
}

bool initializeImu() {
  imuState.state = ImuRunState::INITIALIZING;
  imuState.initialized = false;
  imuState.report_enabled = false;
  imuState.gyro_report_enabled = false;
  imuState.filter_initialized = false;
  imuState.valid = false;
  imuState.gyro_valid = false;
  imuState.stale = false;
  imuState.gyro_stale = false;
  imuState.time_valid = false;
  publishImuSnapshot();

  const bool sdaOk = Wire.setSDA(BNO08X_SDA_PIN);
  const bool sclOk = Wire.setSCL(BNO08X_SCL_PIN);
  if (!sdaOk || !sclOk) {
    ++imuState.init_error_count;
    scheduleInitializationBackoff(micros());
    return false;
  }

  Wire.begin();
  Wire.setClock(BNO08X_I2C_CLOCK_HZ);
  if (!imu.begin(BNO08X_I2C_ADDRESS, Wire, BNO08X_INT_PIN,
                 BNO08X_RST_PIN)) {
    ++imuState.init_error_count;
    scheduleInitializationBackoff(micros());
    return false;
  }

  imuState.initialized = true;
  return enableImuReports();
}

void updateImuRateWindow(uint32_t nowUs) {
  const uint32_t elapsed =
      static_cast<uint32_t>(nowUs - imuState.rate_window_start_us);
  if (elapsed < IMU_RATE_WINDOW_US) {
    return;
  }
  imuState.measured_hz =
      static_cast<float>(imuState.rate_window_samples) * 1000000.0F /
      static_cast<float>(elapsed);
  imuState.gyro_hz =
      static_cast<float>(imuState.rate_window_gyro_samples) * 1000000.0F /
      static_cast<float>(elapsed);
  imuState.rate_window_samples = 0;
  imuState.rate_window_gyro_samples = 0;
  imuState.rate_window_start_us = nowUs;
  imuState.imu_service_window_max_us = 0;

  const uint32_t publishElapsed =
      static_cast<uint32_t>(nowUs - imuState.publish_window_start_us);
  if (publishElapsed >= IMU_RATE_WINDOW_US) {
    imuState.publish_hz =
        static_cast<float>(imuState.publish_window_samples) * 1000000.0F /
        static_cast<float>(publishElapsed);
    imuState.publish_window_samples = 0;
    imuState.publish_window_start_us = nowUs;
  }
}

bool updateImuFreshness(uint32_t nowUs) {
  if (imuState.state == ImuRunState::WAITING_FIRST_SAMPLE) {
    const uint32_t waitingAge =
        static_cast<uint32_t>(nowUs - firstSampleWaitStartUs);
    if (waitingAge > IMU_REINIT_AGE_US) {
      scheduleInitializationBackoff(nowUs);
      return false;
    }
    return true;
  }
  if (imuState.sample_time_us == 0U) {
    return true;
  }
  const uint32_t age = refreshImuSampleAgeUs();
  const bool gyroFresh =
      !ENABLE_GYRO_REPORT ||
      (imuState.gyro_sample_time_us != 0U &&
       imuState.gyro_sample_age_us <= IMU_FRESH_US && imuState.gyro_valid);
  if (age <= IMU_FRESH_US && gyroFresh) {
    imuState.state = ImuRunState::RUNNING;
    imuState.valid = true;
    imuState.stale = false;
    imuState.gyro_stale = false;
    return true;
  }
  imuState.valid = false;
  imuState.stale = true;
  imuState.time_valid = false;
  if (ENABLE_GYRO_REPORT && imuState.gyro_sample_age_us > IMU_FRESH_US) {
    imuState.gyro_valid = false;
    if (!imuState.gyro_stale && imuState.gyro_sample_age_us > IMU_STALE_US) {
      ++imuState.gyro_stale_count;
    }
    imuState.gyro_stale = true;
  }
  if (age <= IMU_STALE_US) {
    if (imuState.state != ImuRunState::STALE) {
      ++imuState.stale_count;
    }
    imuState.state = ImuRunState::STALE;
    return true;
  }
  if (age <= IMU_REINIT_AGE_US) {
    imuState.state = ImuRunState::OFFLINE;
    return true;
  }
  scheduleInitializationBackoff(nowUs);
  return false;
}

void processGameRotationVector(uint64_t eventReceiveTimeUs,
                               uint64_t sensorTimestampUs) {
  const Quaternion sensorQuaternion = {
      imu.sensorValue.un.gameRotationVector.real,
      imu.sensorValue.un.gameRotationVector.i,
      imu.sensorValue.un.gameRotationVector.j,
      imu.sensorValue.un.gameRotationVector.k};
  Quaternion normalizedRaw = {};
  const Quaternion *previousRaw =
      imuState.filter_initialized ? &imuState.raw : nullptr;
  if (!validateAndNormalizeQuaternion(sensorQuaternion, previousRaw,
                                      normalizedRaw)) {
    publishImuSnapshot();
    return;
  }

  Quaternion filtered = normalizedRaw;
  float dtMs = 0.0F;
  if (imuState.filter_initialized && imuState.time_valid) {
    const uint64_t dtUs = eventReceiveTimeUs - imuState.sample_time_us;
    if (dtUs == 0U || dtUs > 100000U) {
      ++imuState.timing_error_count;
    } else {
      const float dtSeconds = static_cast<float>(dtUs) / 1000000.0F;
      dtMs = static_cast<float>(dtUs) / 1000.0F;
      const float alpha = clampFloat(
          1.0F - expf(-2.0F * IMU_PI_F * IMU_QUAT_FILTER_CUTOFF_HZ *
                      dtSeconds),
          0.0F, 1.0F);
      filtered = slerpQuaternion(imuState.filtered, normalizedRaw, alpha);
    }
  }

  Quaternion normalizedFiltered = {};
  if (!normalizeQuaternionUnchecked(filtered, normalizedFiltered)) {
    ++imuState.norm_error_count;
    ++imuState.invalid_count;
    publishImuSnapshot();
    return;
  }

  const uint32_t eventTimeUs32 = static_cast<uint32_t>(eventReceiveTimeUs);
  const uint32_t oldLastValidUs = imuState.last_valid_us;
  imuState.raw = normalizedRaw;
  imuState.filtered = normalizedFiltered;
  imuState.quaternion_norm = quaternionNorm(normalizedRaw);
  imuState.last_dt_ms = dtMs;
  imuState.previous_valid_us = oldLastValidUs;
  imuState.last_valid_us = eventTimeUs32;
  imuState.sample_time_us = eventReceiveTimeUs;
  imuState.sensor_timestamp_us = sensorTimestampUs;
  imuState.sequence += 1U;
  imuState.orientation_accuracy = imu.sensorValue.status;
  imuState.sample_age_us = saturatingAgeUs(time_us_64(), eventReceiveTimeUs);
  ++imuState.sample_count;
  ++imuState.rate_window_samples;
  imuState.filter_initialized = true;
  imuState.valid = true;
  imuState.stale = false;
  imuState.time_valid = true;
  if (!ENABLE_GYRO_REPORT || imuState.gyro_valid) {
    imuState.state = ImuRunState::RUNNING;
  }
  publishImuSnapshot();
}

void processGyro(uint64_t eventReceiveTimeUs, uint64_t sensorTimestampUs) {
  Vector3 gyro = {imu.sensorValue.un.gyroscope.x,
                  imu.sensorValue.un.gyroscope.y,
                  imu.sensorValue.un.gyroscope.z};
  if (!isfinite(gyro.x) || !isfinite(gyro.y) || !isfinite(gyro.z)) {
    ++imuState.gyro_nan_count;
    ++imuState.gyro_invalid_count;
    publishImuSnapshot();
    return;
  }
  if (fabsf(gyro.x) > GYRO_MAX_ABS_RAD_PER_SEC ||
      fabsf(gyro.y) > GYRO_MAX_ABS_RAD_PER_SEC ||
      fabsf(gyro.z) > GYRO_MAX_ABS_RAD_PER_SEC) {
    ++imuState.gyro_invalid_count;
    publishImuSnapshot();
    return;
  }

  imuState.gyro = gyro;
  imuState.gyro_sample_time_us = eventReceiveTimeUs;
  imuState.gyro_sensor_timestamp_us = sensorTimestampUs;
  imuState.gyro_sequence += 1U;
  imuState.gyro_accuracy = imu.sensorValue.status;
  imuState.gyro_sample_age_us = saturatingAgeUs(time_us_64(), eventReceiveTimeUs);
  ++imuState.gyro_sample_count;
  ++imuState.rate_window_gyro_samples;
  imuState.gyro_valid = true;
  imuState.gyro_stale = false;
  if (imuState.valid) {
    imuState.state = ImuRunState::RUNNING;
  }
  publishImuSnapshot();
}

bool serviceBnoEvent(uint64_t eventReceiveTimeUs) {
  const uint8_t sensorId = imu.getSensorEventID();
  if (sensorId == SH2_GAME_ROTATION_VECTOR) {
    processGameRotationVector(eventReceiveTimeUs, imu.sensorValue.timestamp);
    return true;
  }
  if (ENABLE_GYRO_REPORT && sensorId == SH2_GYROSCOPE_CALIBRATED) {
    processGyro(eventReceiveTimeUs, imu.sensorValue.timestamp);
    return true;
  }
  return false;
}

void finishImuTaskTiming(uint32_t taskStartUs) {
  const uint32_t taskDurationUs = static_cast<uint32_t>(micros() - taskStartUs);
  if (taskDurationUs > imuState.sensor_task_max_us) {
    imuState.sensor_task_max_us = taskDurationUs;
    publishImuSnapshot();
  }
}

void imuTask(uint32_t nowUs) {
  const uint32_t taskStartUs = micros();
  updateImuRateWindow(nowUs);
  if (!imuState.initialized) {
    if (imuState.state == ImuRunState::BACKOFF &&
        timeReachedUs(nowUs, nextInitAttemptUs)) {
      (void)initializeImu();
    }
    finishImuTaskTiming(taskStartUs);
    return;
  }
  if (imu.wasReset()) {
    ++imuState.reset_count;
    imuState.valid = false;
    imuState.gyro_valid = false;
    imuState.stale = true;
    imuState.gyro_stale = true;
    imuState.filter_initialized = false;
    imuState.report_enabled = false;
    imuState.gyro_report_enabled = false;
    imuState.time_valid = false;
    (void)enableImuReports();
    finishImuTaskTiming(taskStartUs);
    return;
  }
  if (!imuState.report_enabled ||
      (ENABLE_GYRO_REPORT && !imuState.gyro_report_enabled)) {
    if (reportRetryPending && timeReachedUs(nowUs, nextReportAttemptUs)) {
      (void)enableImuReports();
    }
    finishImuTaskTiming(taskStartUs);
    return;
  }
  if (!updateImuFreshness(nowUs)) {
    finishImuTaskTiming(taskStartUs);
    return;
  }
  if (!timeReachedUs(nowUs, imuState.next_poll_us)) {
    finishImuTaskTiming(taskStartUs);
    return;
  }

  if (digitalRead(BNO08X_INT_PIN) != LOW) {
    advanceImuPollDeadline(nowUs);
    finishImuTaskTiming(taskStartUs);
    return;
  }

  uint8_t drainedEvents = 0;
  const uint32_t drainStartUs = micros();
  bool drainBudgetOverrun = false;
  while (drainedEvents < BNO_EVENT_DRAIN_LIMIT) {
    if (static_cast<uint32_t>(micros() - drainStartUs) >=
        BNO_EVENT_DRAIN_BUDGET_US) {
      drainBudgetOverrun = true;
      break;
    }
    const bool eventReceived = imu.getSensorEvent();
    const uint64_t eventReceiveTimeUs = eventReceived ? time_us_64() : 0U;
    if (!eventReceived) {
      if (drainedEvents == 0U) {
        ++imuState.false_count;
      }
      break;
    }
    ++drainedEvents;
    (void)serviceBnoEvent(eventReceiveTimeUs);
    if (drainedEvents >= BNO_EVENT_DRAIN_LIMIT) {
      break;
    }
    if (digitalRead(BNO08X_INT_PIN) != LOW) {
      break;
    }
    ++imuState.imu_queue_still_asserted_count;
  }
  const uint32_t drainDurationUs = static_cast<uint32_t>(micros() - drainStartUs);
  if (drainDurationUs >= BNO_EVENT_DRAIN_BUDGET_US) {
    drainBudgetOverrun = true;
  }
  imuState.imu_service_last_us = drainDurationUs;
  if (drainDurationUs > imuState.imu_service_window_max_us) {
    imuState.imu_service_window_max_us = drainDurationUs;
  }
  if (drainBudgetOverrun) {
    ++imuState.imu_service_overrun_count;
  }
  imuState.bno_events_per_service = drainedEvents;
  if (drainedEvents > imuState.bno_event_drain_max) {
    imuState.bno_event_drain_max = drainedEvents;
  }
  advanceImuPollDeadline(micros());
  if (drainedEvents == 0U) {
    publishImuSnapshot();
  }
  finishImuTaskTiming(taskStartUs);
}

void echoEdgeHandler(uint8_t sensorIndex) {
  const uint8_t level =
      static_cast<uint8_t>(digitalRead(sensors[sensorIndex].echo_pin));
  const uint64_t edgeTimeUs = time_us_64();
  echoCapture[sensorIndex].last_level = level;
  if (activeSensorForIsr != static_cast<int8_t>(sensorIndex)) {
    ++isrGlitchPending[sensorIndex];
    return;
  }
  if (echoCapturePhase == EchoCapturePhase::WAIT_RISE) {
    if (level == HIGH) {
      echoCapture[sensorIndex].rise_time_us = edgeTimeUs;
      echoCapture[sensorIndex].rise_ready = true;
      echoCapturePhase = EchoCapturePhase::WAIT_FALL;
    } else {
      ++isrGlitchPending[sensorIndex];
    }
    return;
  }
  if (echoCapturePhase == EchoCapturePhase::WAIT_FALL) {
    if (level == LOW) {
      echoCapture[sensorIndex].fall_time_us = edgeTimeUs;
      echoCapture[sensorIndex].fall_ready = true;
      echoCapturePhase = EchoCapturePhase::IGNORE;
    } else {
      ++isrGlitchPending[sensorIndex];
    }
    return;
  }
  ++isrGlitchPending[sensorIndex];
}

void echoIsrU1() { echoEdgeHandler(0); }
void echoIsrU2() { echoEdgeHandler(1); }
void echoIsrU3() { echoEdgeHandler(2); }
void echoIsrU4() { echoEdgeHandler(3); }
void echoIsrU5() { echoEdgeHandler(4); }

void prepareEchoCapture(uint8_t sensorIndex) {
  noInterrupts();
  echoCapture[sensorIndex].rise_time_us = 0;
  echoCapture[sensorIndex].fall_time_us = 0;
  echoCapture[sensorIndex].rise_ready = false;
  echoCapture[sensorIndex].fall_ready = false;
  activeSensorForIsr = static_cast<int8_t>(sensorIndex);
  echoCapturePhase = EchoCapturePhase::WAIT_RISE;
  interrupts();
}

void stopEchoCapture() {
  noInterrupts();
  echoCapturePhase = EchoCapturePhase::IGNORE;
  interrupts();
}

bool consumeEchoRise(uint8_t sensorIndex) {
  noInterrupts();
  const bool ready = echoCapture[sensorIndex].rise_ready;
  echoCapture[sensorIndex].rise_ready = false;
  interrupts();
  return ready;
}

bool consumeEchoFall(uint8_t sensorIndex, uint64_t &riseTimeUs,
                     uint64_t &fallTimeUs) {
  noInterrupts();
  const bool ready = echoCapture[sensorIndex].fall_ready;
  if (ready) {
    riseTimeUs = echoCapture[sensorIndex].rise_time_us;
    fallTimeUs = echoCapture[sensorIndex].fall_time_us;
    echoCapture[sensorIndex].fall_ready = false;
  }
  interrupts();
  return ready;
}

void synchronizeGlitchCounters() {
  noInterrupts();
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint32_t pending = isrGlitchPending[i];
    isrGlitchPending[i] = 0;
    sensors[i].glitch_count += pending;
  }
  interrupts();
}

void enterQuietGap(uint32_t nowUs) {
  digitalWrite(sensors[activeSensorIndex].trig_pin, LOW);
  stopEchoCapture();
  scanState = ScanState::QUIET_GAP;
  stateStartUs = nowUs;
}

void updateMissingHealth(UltrasonicSensor &sensor, uint64_t nowUs) {
  const uint32_t ageUs =
      sensor.filter_initialized
          ? saturatingAgeUs(nowUs, sensor.raw_sample_time_us)
          : saturatingAgeUs(nowUs, scanWindowStartUs);
  if (!sensor.filter_initialized) {
    sensor.safety_valid = false;
    sensor.health =
        (sensor.consecutive_timeout_count >= 3U || ageUs > HOLD_MAX_AGE_US)
            ? SensorHealth::UNKNOWN
            : SensorHealth::INIT;
    return;
  }
  if (sensor.consecutive_timeout_count >= 3U || ageUs > HOLD_MAX_AGE_US) {
    sensor.health = SensorHealth::UNKNOWN;
    sensor.safety_valid = false;
    return;
  }
  sensor.health = SensorHealth::HOLD;
  sensor.safety_valid = true;
}

void updateFilterFromValidRaw(UltrasonicSensor &sensor, float distanceCm,
                              uint64_t sampleTimeUs) {
  const bool recoverFromUnknown =
      sensor.filter_initialized && sensor.health == SensorHealth::UNKNOWN;
  sensor.raw_cm = distanceCm;
  sensor.raw_sample_time_us = sampleTimeUs;
  ++sensor.raw_sequence;
  sensor.raw_valid = true;
  sensor.timeout = false;
  sensor.consecutive_timeout_count = 0;
  if (!sensor.filter_initialized || recoverFromUnknown) {
    if (recoverFromUnknown) {
      ++sensor.filter_reset_count;
    }
    sensor.valid_history[0] = distanceCm;
    sensor.valid_history[1] = 0.0F;
    sensor.valid_history[2] = 0.0F;
    sensor.history_count = 1;
    sensor.history_index = 1;
    sensor.median_cm = distanceCm;
    sensor.filtered_cm = distanceCm;
    sensor.safety_cm = distanceCm;
    sensor.filter_initialized = true;
  } else {
    sensor.valid_history[sensor.history_index] = distanceCm;
    sensor.history_index = static_cast<uint8_t>((sensor.history_index + 1U) % 3U);
    if (sensor.history_count < 3U) {
      ++sensor.history_count;
    }
    sensor.median_cm = sensor.history_count < 3U
                           ? distanceCm
                           : medianOfThree(sensor.valid_history[0],
                                           sensor.valid_history[1],
                                           sensor.valid_history[2]);
    if (sensor.history_count == 3U &&
        fabsf(distanceCm - sensor.median_cm) > MEDIAN_REJECT_DIAGNOSTIC_CM) {
      ++sensor.median_reject_count;
    }
    sensor.filtered_cm =
        clampDistance(sensor.filtered_cm +
                      ULTRASONIC_EMA_ALPHA *
                          (sensor.median_cm - sensor.filtered_cm));
    sensor.safety_cm = distanceCm < sensor.filtered_cm ? distanceCm
                                                       : sensor.filtered_cm;
  }
  sensor.filter_sample_time_us = sampleTimeUs;
  ++sensor.filter_sequence;
  sensor.safety_valid = true;
  sensor.health = SensorHealth::VALID;
}

void recordTimeout(uint32_t nowUs) {
  UltrasonicSensor &sensor = sensors[activeSensorIndex];
  const uint64_t updateTimeUs = time_us_64();
  sensor.pulse_width_us = 0;
  sensor.raw_valid = false;
  sensor.timeout = true;
  sensor.last_update_us = updateTimeUs;
  ++sensor.timeout_count;
  if (sensor.consecutive_timeout_count < UINT32_MAX) {
    ++sensor.consecutive_timeout_count;
  }
  updateMissingHealth(sensor, updateTimeUs);
  enterQuietGap(nowUs);
}

void recordInvalidEcho(UltrasonicSensor &sensor, uint64_t updateTimeUs) {
  sensor.raw_valid = false;
  sensor.timeout = false;
  sensor.last_update_us = updateTimeUs;
  updateMissingHealth(sensor, updateTimeUs);
}

void recordEcho(uint64_t riseTimeUs, uint64_t fallTimeUs, uint32_t nowUs) {
  UltrasonicSensor &sensor = sensors[activeSensorIndex];
  sensor.timeout = false;
  sensor.last_update_us = fallTimeUs;
  if (fallTimeUs <= riseTimeUs) {
    ++sensor.glitch_count;
    recordInvalidEcho(sensor, fallTimeUs);
    enterQuietGap(nowUs);
    return;
  }
  const uint64_t pulseWidthUs64 = fallTimeUs - riseTimeUs;
  sensor.pulse_width_us =
      pulseWidthUs64 > static_cast<uint64_t>(UINT32_MAX)
          ? UINT32_MAX
          : static_cast<uint32_t>(pulseWidthUs64);
  const float distanceCm =
      static_cast<float>(sensor.pulse_width_us) * SOUND_CM_PER_US / 2.0F;
  if (distanceCm < MIN_DISTANCE_CM || distanceCm > MAX_DISTANCE_CM) {
    ++sensor.range_error_count;
    recordInvalidEcho(sensor, fallTimeUs);
    enterQuietGap(nowUs);
    return;
  }
  ++sensor.valid_count;
  updateFilterFromValidRaw(sensor, distanceCm, fallTimeUs);
  enterQuietGap(nowUs);
}

void refreshSensorHealth() {
  const uint64_t nowUs = time_us_64();
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    UltrasonicSensor &sensor = sensors[i];
    const uint32_t ageUs =
        sensor.filter_initialized
            ? saturatingAgeUs(nowUs, sensor.raw_sample_time_us)
            : saturatingAgeUs(nowUs, scanWindowStartUs);
    if (!sensor.filter_initialized) {
      sensor.safety_valid = false;
      sensor.health =
          (sensor.consecutive_timeout_count >= 3U || ageUs > HOLD_MAX_AGE_US)
              ? SensorHealth::UNKNOWN
              : SensorHealth::INIT;
      continue;
    }
    if (sensor.consecutive_timeout_count >= 3U || ageUs > HOLD_MAX_AGE_US) {
      sensor.health = SensorHealth::UNKNOWN;
      sensor.safety_valid = false;
    } else if (!sensor.raw_valid) {
      sensor.health = SensorHealth::HOLD;
      sensor.safety_valid = true;
    }
  }
}

void ultrasonicTask() {
  const uint32_t nowUs = micros();
  UltrasonicSensor &sensor = sensors[activeSensorIndex];
  switch (scanState) {
  case ScanState::SELECT_SENSOR:
    digitalWrite(sensor.trig_pin, LOW);
    activeSensorForIsr = static_cast<int8_t>(activeSensorIndex);
    scanState = ScanState::TRIGGER_LOW;
    stateStartUs = nowUs;
    sensorSlotStartUs = nowUs;
    break;
  case ScanState::TRIGGER_LOW:
    if (elapsedUs(nowUs, stateStartUs, TRIGGER_LOW_US)) {
      digitalWrite(sensor.trig_pin, HIGH);
      scanState = ScanState::TRIGGER_HIGH;
      stateStartUs = nowUs;
    }
    break;
  case ScanState::TRIGGER_HIGH:
    if (elapsedUs(nowUs, stateStartUs, TRIGGER_HIGH_US)) {
      const uint32_t highUs = static_cast<uint32_t>(nowUs - stateStartUs);
      if (highUs > triggerHighMaxUs) {
        triggerHighMaxUs = highUs;
      }
      prepareEchoCapture(activeSensorIndex);
      digitalWrite(sensor.trig_pin, LOW);
      echoWaitStartUs = nowUs;
      scanState = ScanState::WAIT_ECHO_RISE;
    }
    break;
  case ScanState::WAIT_ECHO_RISE:
    if (consumeEchoRise(activeSensorIndex)) {
      scanState = ScanState::WAIT_ECHO_FALL;
    } else if (elapsedUs(nowUs, echoWaitStartUs, ECHO_TIMEOUT_US)) {
      recordTimeout(nowUs);
    }
    break;
  case ScanState::WAIT_ECHO_FALL: {
    uint64_t riseTimeUs = 0;
    uint64_t fallTimeUs = 0;
    if (consumeEchoFall(activeSensorIndex, riseTimeUs, fallTimeUs)) {
      recordEcho(riseTimeUs, fallTimeUs, nowUs);
    } else if (elapsedUs(nowUs, echoWaitStartUs, ECHO_TIMEOUT_US)) {
      recordTimeout(nowUs);
    }
    break;
  }
  case ScanState::QUIET_GAP:
    if (elapsedUs(nowUs, stateStartUs, QUIET_GAP_US) &&
        elapsedUs(nowUs, sensorSlotStartUs, SENSOR_SLOT_US)) {
      scanState = ScanState::NEXT_SENSOR;
    }
    break;
  case ScanState::NEXT_SENSOR:
    if (activeSensorIndex == SENSOR_COUNT - 1U) {
      ++scansInWindow;
    }
    activeSensorIndex =
        static_cast<uint8_t>((activeSensorIndex + 1U) % SENSOR_COUNT);
    scanState = ScanState::SELECT_SENSOR;
    break;
  }
}

void updateScanRate() {
  const uint64_t nowUs = time_us_64();
  const uint64_t elapsedWindowUs = nowUs - scanWindowStartUs;
  if (elapsedWindowUs < ULTRASONIC_RATE_WINDOW_US) {
    return;
  }
  scanHz = static_cast<float>(scansInWindow) * 1000000.0F /
           static_cast<float>(elapsedWindowUs);
  scansInWindow = 0;
  scanWindowStartUs = nowUs;
}

bool triggerPulseActive() {
  return scanState == ScanState::TRIGGER_LOW ||
         scanState == ScanState::TRIGGER_HIGH;
}

void assignString(rosidl_runtime_c__String &dst, char *data) {
  dst.data = data;
  dst.size = strlen(data);
  dst.capacity = dst.size + 1U;
}

void writeString(rosidl_runtime_c__String &dst, char *buffer, size_t capacity,
                 const char *text) {
  if (capacity == 0U) {
    return;
  }
  snprintf(buffer, capacity, "%s", text);
  dst.data = buffer;
  dst.size = strlen(buffer);
  dst.capacity = capacity;
}

void setHeader(std_msgs__msg__Header &header, char *frameId,
               uint64_t sampleTimeUs) {
  assignString(header.frame_id, frameId);
  int64_t stampNs = 0;
  if (timeSynchronized && sampleTimeUs != 0U) {
    stampNs = static_cast<int64_t>(sampleTimeUs * 1000ULL) + epochOffsetNs;
    if (stampNs <= lastRosStampNs) {
      stampNs = lastRosStampNs + 1;
    }
    lastRosStampNs = stampNs;
  }
  header.stamp.sec = static_cast<int32_t>(stampNs / 1000000000LL);
  header.stamp.nanosec =
      static_cast<uint32_t>(stampNs % 1000000000LL);
}

bool publishChecked(rcl_publisher_t *publisher, const void *message) {
  const rcl_ret_t ret = rcl_publish(publisher, message, nullptr);
  if (ret != RCL_RET_OK) {
    ++publishErrorCount;
    rosSessionError = true;
    return false;
  }
  return true;
}

bool publishDiagnosticChecked() {
  const rcl_ret_t ret = rcl_publish(&diagnosticsPub, &diagnosticMsg, nullptr);
  if (ret != RCL_RET_OK) {
    ++diagnosticPublishError;
    return false;
  }
  return true;
}

rcl_ret_t initBestEffortPublisher(rcl_publisher_t &publisher,
                                  const rosidl_message_type_support_t *ts,
                                  const char *topicName) {
  rcl_publisher_options_t options = rcl_publisher_get_default_options();
  options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  options.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  options.qos.depth = 1;
  return rcl_publisher_init(&publisher, &node, ts, topicName, &options);
}

void finiPublisher(rcl_publisher_t &publisher) {
  if (publisher.impl != nullptr) {
    const rcl_ret_t ret = rcl_publisher_fini(&publisher, &node);
    (void)ret;
  }
  publisher = rcl_get_zero_initialized_publisher();
}

void zeroEntities() {
  support = {};
  node = rcl_get_zero_initialized_node();
  heartbeatPub = rcl_get_zero_initialized_publisher();
  imuPub = rcl_get_zero_initialized_publisher();
  obstaclePub = rcl_get_zero_initialized_publisher();
  diagnosticsPub = rcl_get_zero_initialized_publisher();
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    rangePubs[i] = rcl_get_zero_initialized_publisher();
  }
}

void destroyEntities() {
  if constexpr (ENABLE_ROS_DIAGNOSTICS) {
    finiPublisher(diagnosticsPub);
  }
  if constexpr (ENABLE_ULTRASONICS && ENABLE_ROS_RANGE_PUBLISHERS) {
    finiPublisher(obstaclePub);
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      finiPublisher(rangePubs[i]);
    }
  }
  if constexpr (ENABLE_IMU && ENABLE_ROS_IMU_PUBLISHER) {
    finiPublisher(imuPub);
  }
  finiPublisher(heartbeatPub);
  if (node.impl != nullptr) {
    const rcl_ret_t ret = rcl_node_fini(&node);
    (void)ret;
  }
  if (support.context.impl != nullptr) {
    (void)rclc_support_fini(&support);
  }
  zeroEntities();
  entitiesCreated = false;
  timeSynchronized = false;
  rosSessionError = false;
  runningAgentPingFailures = 0;
}

bool createEntities() {
  zeroEntities();
  entityCreateErrorStep = 0;
  rosSessionError = false;
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t rc = rcl_init_options_init(&init_options, allocator);
  if (rc == RCL_RET_OK) {
    rc = rcl_init_options_set_domain_id(&init_options, PICO_A_ROS_DOMAIN_ID);
  }
  if (rc == RCL_RET_OK) {
    rc = rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator);
  }
  (void)rcl_init_options_fini(&init_options);
  if (rc != RCL_RET_OK) {
    if (support.context.impl != nullptr) {
      (void)rclc_support_fini(&support);
    }
    entityCreateErrorStep = 1;
    return false;
  }
  setBootStage(BootStage::ALLOCATOR_SUPPORT_READY);
  if (rclc_node_init_default(&node, "pico_a_sensor_node", "",
                             &support) != RCL_RET_OK) {
    entityCreateErrorStep = 2;
    return false;
  }
  setBootStage(BootStage::NODE_READY);
  if (initBestEffortPublisher(
          heartbeatPub, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
          "/pico_a/heartbeat") != RCL_RET_OK) {
    entityCreateErrorStep = 3;
    return false;
  }
  if constexpr (ENABLE_IMU && ENABLE_ROS_IMU_PUBLISHER) {
    if (initBestEffortPublisher(
            imuPub, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "/imu/data") != RCL_RET_OK) {
      entityCreateErrorStep = 4;
      return false;
    }
  }
  if constexpr (ENABLE_ULTRASONICS && ENABLE_ROS_RANGE_PUBLISHERS) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      if (initBestEffortPublisher(
              rangePubs[i], ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
              rangeTopics[i]) != RCL_RET_OK) {
        entityCreateErrorStep = static_cast<uint32_t>(10U + i);
        return false;
      }
    }
    if (initBestEffortPublisher(
            obstaclePub, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "/pico_a/obstacle_stop") != RCL_RET_OK) {
      entityCreateErrorStep = 20;
      return false;
    }
  }
  if constexpr (ENABLE_ROS_DIAGNOSTICS) {
    if (initBestEffortPublisher(
            diagnosticsPub,
            ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
            "/diagnostics") != RCL_RET_OK) {
      entityCreateErrorStep = 30;
      return false;
    }
  }
  setBootStage(BootStage::PUBLISHERS_READY);
  lastRunningAgentPingMs = millis();
  runningAgentPingFailures = 0;
  entitiesCreated = true;
  return true;
}

void initRangeMessages() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    rangeMsgs[i].radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    rangeMsgs[i].field_of_view = 0.261799F;
    rangeMsgs[i].min_range = MIN_DISTANCE_CM / 100.0F;
    rangeMsgs[i].max_range = MAX_DISTANCE_CM / 100.0F;
    rangeMsgs[i].variance = 0.0004F;
    setHeader(rangeMsgs[i].header, rangeFrames[i], 0);
  }
}

void initImuMessage() {
  imuMsg = sensor_msgs__msg__Imu{};
  setHeader(imuMsg.header, frameImu, 0);
  for (uint8_t i = 0; i < 9U; ++i) {
    imuMsg.orientation_covariance[i] = 0.0;
    imuMsg.angular_velocity_covariance[i] = 0.0;
    imuMsg.linear_acceleration_covariance[i] = 0.0;
  }
  imuMsg.orientation_covariance[0] = IMU_ORIENTATION_VARIANCE;
  imuMsg.orientation_covariance[4] = IMU_ORIENTATION_VARIANCE;
  imuMsg.orientation_covariance[8] = IMU_ORIENTATION_VARIANCE;
  imuMsg.angular_velocity_covariance[0] = IMU_GYRO_VARIANCE;
  imuMsg.angular_velocity_covariance[4] = IMU_GYRO_VARIANCE;
  imuMsg.angular_velocity_covariance[8] = IMU_GYRO_VARIANCE;
  imuMsg.linear_acceleration_covariance[0] = -1.0;
}

void initDiagnostics() {
  memset(&diagnosticBuffers, 0, sizeof(diagnosticBuffers));
  diagnosticMsg.status.data = diagnosticBuffers.statuses;
  diagnosticMsg.status.size = 1;
  diagnosticMsg.status.capacity = 3;
  setHeader(diagnosticMsg.header, frameDiag, 0);

  for (uint8_t i = 0; i < 3U; ++i) {
    diagnosticBuffers.statuses[i].values.data = diagnosticBuffers.values_kv;
    diagnosticBuffers.statuses[i].values.size = 0;
    diagnosticBuffers.statuses[i].values.capacity = 6;
    assignString(diagnosticBuffers.statuses[i].hardware_id, diagHardware);
  }
}

void setDiagValue(diagnostic_msgs__msg__KeyValue &kv, uint8_t bufferIndex,
                  const char *value) {
  writeString(kv.value, diagnosticBuffers.values[bufferIndex],
              sizeof(diagnosticBuffers.values[bufferIndex]), value);
}

void setDiagValueU32(diagnostic_msgs__msg__KeyValue &kv, uint8_t bufferIndex,
                     uint32_t value) {
  snprintf(diagnosticBuffers.values[bufferIndex],
           sizeof(diagnosticBuffers.values[bufferIndex]), "%" PRIu32, value);
  assignString(kv.value, diagnosticBuffers.values[bufferIndex]);
}

void setDiagValueFloat(diagnostic_msgs__msg__KeyValue &kv, uint8_t bufferIndex,
                       float value, uint8_t decimals) {
  char format[8];
  snprintf(format, sizeof(format), "%%.%uf", decimals);
  snprintf(diagnosticBuffers.values[bufferIndex],
           sizeof(diagnosticBuffers.values[bufferIndex]), format,
           static_cast<double>(value));
  assignString(kv.value, diagnosticBuffers.values[bufferIndex]);
}

void setDiagKey(uint8_t index, char *key) {
  assignString(diagnosticBuffers.values_kv[index].key, key);
}

void setDiagPageValue(uint8_t index, const char *value) {
  setDiagValue(diagnosticBuffers.values_kv[index], index, value);
}

void setDiagPageValueU32(uint8_t index, uint32_t value) {
  setDiagValueU32(diagnosticBuffers.values_kv[index], index, value);
}

void setDiagPageValueFloat(uint8_t index, float value, uint8_t decimals) {
  setDiagValueFloat(diagnosticBuffers.values_kv[index], index, value, decimals);
}

diagnostic_msgs__msg__DiagnosticStatus &prepareDiagnosticPage(char *name,
                                                              uint8_t level,
                                                              char *message) {
  diagnosticMsg.status.size = 1;
  for (uint8_t i = 0; i < 3U; ++i) {
    diagnosticBuffers.statuses[i].values.size = 0;
  }
  diagnostic_msgs__msg__DiagnosticStatus &status =
      diagnosticBuffers.statuses[0];
  assignString(status.name, name);
  assignString(status.hardware_id, diagHardware);
  status.level = level;
  assignString(status.message, message);
  status.values.data = diagnosticBuffers.values_kv;
  status.values.size = 6;
  status.values.capacity = 6;
  setDiagKey(0, keyPage);
  return status;
}

void setupDiagnosticsForPublish() {
  setHeader(diagnosticMsg.header, frameDiag, time_us_64());

  ImuSnapshot snapshot = {};
  const bool gotSnapshot = readImuSnapshot(snapshot);
  const uint64_t nowUs = time_us_64();
  const uint32_t orientationAgeUs =
      gotSnapshot ? saturatingAgeUs(nowUs, snapshot.sample_time_us) : UINT32_MAX;
  const uint32_t gyroAgeUs =
      gotSnapshot ? saturatingAgeUs(nowUs, snapshot.gyro_sample_time_us)
                  : UINT32_MAX;
  const bool imuOk = gotSnapshot && snapshot.valid &&
                     (!ENABLE_GYRO_REPORT || snapshot.gyro_valid) &&
                     orientationAgeUs <= IMU_PUBLISH_MAX_AGE_US &&
                     (!ENABLE_GYRO_REPORT ||
                      gyroAgeUs <= IMU_PUBLISH_MAX_AGE_US);
  const uint8_t imuLevel =
      imuOk ? static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK)
            : static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
  const uint8_t microLevel =
      entitiesCreated
          ? static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK)
          : static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__ERROR);
  bool ultrasonicUnknown = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (!sensors[i].safety_valid &&
        (sensors[i].health == SensorHealth::INIT ||
         sensors[i].health == SensorHealth::UNKNOWN)) {
      ultrasonicUnknown = true;
    }
  }
  const uint8_t ultrasonicLevel =
      ENABLE_ULTRASONICS && ultrasonicUnknown
          ? static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN)
          : static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  char *ultrasonicMessage =
      !ENABLE_ULTRASONICS ? diagMsgDisabled
                          : (ultrasonicUnknown ? diagMsgSensorUnknown : diagMsgOk);

  switch (diagnosticPage) {
  case 0:
    (void)prepareDiagnosticPage(diagNameImu, imuLevel,
                                imuOk ? diagMsgOk : diagMsgWarn);
    setDiagPageValue(0, diagPageImuRates);
    setDiagKey(1, keyImuOrientationRawHz);
    setDiagPageValueFloat(1, snapshot.measured_hz, 1);
    setDiagKey(2, keyImuGyroRawHz);
    setDiagPageValueFloat(2, snapshot.gyro_hz, 1);
    setDiagKey(3, keyImuPublishHz);
    setDiagPageValueFloat(3, imuPublishHz, 1);
    setDiagKey(4, keyOrientationSequence);
    setDiagPageValueU32(4, snapshot.sequence);
    setDiagKey(5, keyGyroSequence);
    setDiagPageValueU32(5, snapshot.gyro_sequence);
    break;
  case 1:
    (void)prepareDiagnosticPage(diagNameImu, imuLevel,
                                imuOk ? diagMsgOk : diagMsgWarn);
    setDiagPageValue(0, diagPageImuTiming);
    setDiagKey(1, keyOrientationAgeUs);
    setDiagPageValueU32(1, orientationAgeUs);
    setDiagKey(2, keyGyroAgeUs);
    setDiagPageValueU32(2, gyroAgeUs);
    setDiagKey(3, keyOrientationGyroSkewUs);
    setDiagPageValueU32(3, imuPairSkewUs);
    setDiagKey(4, keyBnoEventsPerService);
    setDiagPageValueU32(4, snapshot.bno_events_per_service);
    setDiagKey(5, keyBnoEventDrainMax);
    setDiagPageValueU32(5, snapshot.bno_event_drain_max);
    break;
  case 2:
    (void)prepareDiagnosticPage(diagNameImu, imuLevel,
                                imuOk ? diagMsgOk : diagMsgWarn);
    setDiagPageValue(0, diagPageImuRejectA);
    setDiagKey(1, keyRejectOrientationNotNew);
    setDiagPageValueU32(1, rejectOrientationNotNew);
    setDiagKey(2, keyRejectGyroNotNew);
    setDiagPageValueU32(2, rejectGyroNotNew);
    setDiagKey(3, keyRejectFreshness);
    setDiagPageValueU32(3, rejectFreshness);
    setDiagKey(4, keyRejectSkew);
    setDiagPageValueU32(4, rejectSkew);
    setDiagKey(5, keyRejectInvalid);
    setDiagPageValueU32(5, rejectInvalid);
    break;
  case 3:
    (void)prepareDiagnosticPage(diagNameImu, imuLevel,
                                imuOk ? diagMsgOk : diagMsgWarn);
    setDiagPageValue(0, diagPageImuRejectB);
    setDiagKey(1, keyRejectRosTime);
    setDiagPageValueU32(1, rejectRosTime);
    setDiagKey(2, keyPublishError);
    setDiagPageValueU32(2, diagnosticPublishError);
    setDiagKey(3, keySensorTaskMaxUs);
    setDiagPageValueU32(3, snapshot.sensor_task_max_us);
    setDiagKey(4, keyLoopMaxUs);
    setDiagPageValueU32(4, loopMaxUs);
    setDiagKey(5, keyReconnect);
    setDiagPageValueU32(5, reconnectCount);
    break;
  case 4:
    (void)prepareDiagnosticPage(diagNameImu, imuLevel,
                                imuOk ? diagMsgOk : diagMsgWarn);
    setDiagPageValue(0, diagPageImuService);
    setDiagKey(1, keyImuServiceLastUs);
    setDiagPageValueU32(1, snapshot.imu_service_last_us);
    setDiagKey(2, keyImuServiceWindowMaxUs);
    setDiagPageValueU32(2, snapshot.imu_service_window_max_us);
    setDiagKey(3, keyImuServiceOverrunCount);
    setDiagPageValueU32(3, snapshot.imu_service_overrun_count);
    setDiagKey(4, keyImuPollMissCount);
    setDiagPageValueU32(4, snapshot.imu_poll_miss_count);
    setDiagKey(5, keyImuQueueStillAssertedCount);
    setDiagPageValueU32(5, snapshot.imu_queue_still_asserted_count);
    break;
  case 5:
    (void)prepareDiagnosticPage(diagNameUltrasonic, ultrasonicLevel,
                                ultrasonicMessage);
    setDiagPageValue(0, diagPageUltrasonicA);
    setDiagKey(1, keyScanHz);
    setDiagPageValueFloat(1, scanHz, 1);
    for (uint8_t i = 0; i < 4U; ++i) {
      setDiagKey(2 + i, ultraKeyState[i]);
      setDiagPageValue(2 + i, sensorHealthName(sensors[i].health));
    }
    break;
  case 6:
    (void)prepareDiagnosticPage(diagNameUltrasonic, ultrasonicLevel,
                                ultrasonicMessage);
    setDiagPageValue(0, diagPageUltrasonicB);
    setDiagKey(1, ultraKeyState[4]);
    setDiagPageValue(1, sensorHealthName(sensors[4].health));
    for (uint8_t i = 0; i < 4U; ++i) {
      setDiagKey(2 + i, ultraKeyRange[i]);
      setDiagPageValueFloat(2 + i, sensors[i].safety_cm, 1);
    }
    break;
  case 7:
    (void)prepareDiagnosticPage(diagNameUltrasonic, ultrasonicLevel,
                                ultrasonicMessage);
    setDiagPageValue(0, diagPageUltrasonicC);
    setDiagKey(1, ultraKeyRange[4]);
    setDiagPageValueFloat(1, sensors[4].safety_cm, 1);
    for (uint8_t i = 0; i < 4U; ++i) {
      setDiagKey(2 + i, ultraKeyTimeout[i]);
      setDiagPageValueU32(2 + i, sensors[i].timeout_count);
    }
    break;
  default:
    (void)prepareDiagnosticPage(diagNameMicroRos, microLevel,
                                entitiesCreated ? diagMsgOk : diagMsgError);
    setDiagPageValue(0, diagPageMicroRos);
    setDiagKey(1, keyAgentState);
    setDiagPageValue(1, entitiesCreated ? "RUN" : "WAIT_AGENT");
    setDiagKey(2, keyTimeSynced);
    setDiagPageValue(2, timeSynchronized ? "true" : "false");
    setDiagKey(3, keyPublishErrors);
    setDiagPageValueU32(3, publishErrorCount);
    setDiagKey(4, keyReconnects);
    setDiagPageValueU32(4, reconnectCount);
    setDiagKey(5, keyBootStage);
    setDiagPageValueU32(5, bootMarker.current_stage);
    break;
  }
}

void publishHeartbeat(uint32_t nowMs) {
  if (static_cast<uint32_t>(nowMs - lastHeartbeatMs) < HEARTBEAT_PERIOD_MS) {
    return;
  }
  lastHeartbeatMs = nowMs;
  heartbeatMsg.data = heartbeatCount++;
  (void)publishChecked(&heartbeatPub, &heartbeatMsg);
}

void publishImuIfReady() {
  if constexpr (!ENABLE_IMU || !ENABLE_ROS_IMU_PUBLISHER) {
    return;
  }
  ImuSnapshot snapshot = {};
  if (!readImuSnapshot(snapshot)) {
    if (!rejectSnapshotReadCounted) {
      ++rejectInvalid;
      rejectSnapshotReadCounted = true;
    }
    return;
  }
  rejectSnapshotReadCounted = false;
  static uint32_t lastPublishedSequence = 0;
  if (snapshot.sequence == lastPublishedSequence) {
    countUniqueOrientationReject(rejectOrientationNotNew,
                                 lastRejectOrientationNotNewSequence,
                                 snapshot.sequence);
    return;
  }
  if (!timeSynchronized) {
    countUniqueOrientationReject(rejectRosTime, lastRejectRosTimeSequence,
                                 snapshot.sequence);
    return;
  }
  const uint64_t nowUs = time_us_64();
  const uint32_t ageUs = saturatingAgeUs(nowUs, snapshot.sample_time_us);
  const uint32_t gyroAgeUs = saturatingAgeUs(nowUs, snapshot.gyro_sample_time_us);
  const uint32_t pairSkewUs =
      snapshot.sample_time_us > snapshot.gyro_sample_time_us
          ? saturatingAgeUs(snapshot.sample_time_us, snapshot.gyro_sample_time_us)
          : saturatingAgeUs(snapshot.gyro_sample_time_us, snapshot.sample_time_us);
  const bool orientationFinite =
      isfinite(snapshot.filtered.w) && isfinite(snapshot.filtered.x) &&
      isfinite(snapshot.filtered.y) && isfinite(snapshot.filtered.z);
  const bool gyroFinite =
      !ENABLE_GYRO_REPORT ||
      (isfinite(snapshot.gyro.x) && isfinite(snapshot.gyro.y) &&
       isfinite(snapshot.gyro.z));
  if (!snapshot.valid || !orientationFinite ||
      (ENABLE_GYRO_REPORT && (!snapshot.gyro_valid || !gyroFinite))) {
    countUniqueOrientationReject(rejectInvalid, lastRejectInvalidSequence,
                                 snapshot.sequence);
    return;
  }
  if (ageUs > IMU_PUBLISH_MAX_AGE_US ||
      (ENABLE_GYRO_REPORT && gyroAgeUs > IMU_PUBLISH_MAX_AGE_US)) {
    countUniqueOrientationReject(rejectFreshness, lastRejectFreshnessSequence,
                                 snapshot.sequence);
    return;
  }
  if (ENABLE_GYRO_REPORT && pairSkewUs > IMU_PAIR_MAX_SKEW_US) {
    countUniqueOrientationReject(rejectSkew, lastRejectSkewSequence,
                                 snapshot.sequence);
    if (pairSkewUs > imuPairSkewMaxUs) {
      imuPairSkewMaxUs = pairSkewUs;
    }
    return;
  }
  lastPublishedSequence = snapshot.sequence;
  setHeader(imuMsg.header, frameImu, snapshot.sample_time_us);
  imuMsg.orientation.w = snapshot.filtered.w;
  imuMsg.orientation.x = snapshot.filtered.x;
  imuMsg.orientation.y = snapshot.filtered.y;
  imuMsg.orientation.z = snapshot.filtered.z;
  imuMsg.angular_velocity.x = ENABLE_GYRO_REPORT ? snapshot.gyro.x : 0.0;
  imuMsg.angular_velocity.y = ENABLE_GYRO_REPORT ? snapshot.gyro.y : 0.0;
  imuMsg.angular_velocity.z = ENABLE_GYRO_REPORT ? snapshot.gyro.z : 0.0;
  imuMsg.linear_acceleration.x = 0.0;
  imuMsg.linear_acceleration.y = 0.0;
  imuMsg.linear_acceleration.z = 0.0;
  imuPairSkewUs = pairSkewUs;
  if (pairSkewUs > imuPairSkewMaxUs) {
    imuPairSkewMaxUs = pairSkewUs;
  }
  ++imuPairPublishCount;
  ++imuPublishWindowSamples;
  if (imuPublishWindowStartUs == 0U) {
    imuPublishWindowStartUs = static_cast<uint32_t>(nowUs);
  } else {
    const uint32_t elapsed =
        static_cast<uint32_t>(static_cast<uint32_t>(nowUs) -
                              imuPublishWindowStartUs);
    if (elapsed >= IMU_RATE_WINDOW_US) {
      imuPublishHz = static_cast<float>(imuPublishWindowSamples) *
                     1000000.0F / static_cast<float>(elapsed);
      imuPublishWindowSamples = 0;
      imuPublishWindowStartUs = static_cast<uint32_t>(nowUs);
    }
  }
  (void)publishChecked(&imuPub, &imuMsg);
}

void publishRanges() {
  if constexpr (!ENABLE_ULTRASONICS || !ENABLE_ROS_RANGE_PUBLISHERS) {
    return;
  }
  const uint64_t nowUs = time_us_64();
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (!sensors[i].safety_valid || !sensors[i].filter_initialized) {
      continue;
    }
    const uint32_t sequence = sensors[i].filter_sequence;
    if (sequence == 0U || sequence == lastRangePublishSeq[i]) {
      continue;
    }
    if (lastRangePublishUs[i] != 0U &&
        nowUs - lastRangePublishUs[i] < RANGE_PUBLISH_MIN_PERIOD_US) {
      continue;
    }
    setHeader(rangeMsgs[i].header, rangeFrames[i],
              sensors[i].filter_sample_time_us);
    rangeMsgs[i].range = sensors[i].safety_cm / 100.0F;
    if (publishChecked(&rangePubs[i], &rangeMsgs[i])) {
      lastRangePublishSeq[i] = sequence;
      lastRangePublishUs[i] = nowUs;
    }
  }
}

void publishObstacle(uint32_t nowMs) {
  if constexpr (!ENABLE_ULTRASONICS || !ENABLE_ROS_RANGE_PUBLISHERS) {
    return;
  }
  if (static_cast<uint32_t>(nowMs - lastObstacleMs) < OBSTACLE_PERIOD_MS) {
    return;
  }
  lastObstacleMs = nowMs;
  bool stop = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (!sensors[i].safety_valid || sensors[i].health == SensorHealth::UNKNOWN) {
      stop = true;
    } else if (sensors[i].safety_cm <= OBSTACLE_STOP_CM[i]) {
      stop = true;
    }
  }
  obstacleMsg.data = stop;
  (void)publishChecked(&obstaclePub, &obstacleMsg);
}

void publishDiagnostics(uint32_t nowMs) {
  if constexpr (!ENABLE_ROS_DIAGNOSTICS) {
    return;
  }
  if (static_cast<uint32_t>(nowMs - lastDiagnosticMs) < DIAGNOSTIC_PERIOD_MS) {
    return;
  }
  lastDiagnosticMs = nowMs;
  setupDiagnosticsForPublish();
  (void)publishDiagnosticChecked();
  if constexpr (ENABLE_IMU && ENABLE_ULTRASONICS) {
    diagnosticPage = diagnosticPage >= 8U ? 0U : diagnosticPage + 1U;
  } else if constexpr (ENABLE_IMU) {
    diagnosticPage = diagnosticPage >= 4U ? 0U : diagnosticPage + 1U;
  } else if constexpr (ENABLE_ULTRASONICS) {
    diagnosticPage = diagnosticPage < 5U || diagnosticPage >= 8U
                         ? 5U
                         : diagnosticPage + 1U;
  } else {
    diagnosticPage = 8U;
  }
}

bool checkRunningAgent(uint32_t nowMs) {
  if (static_cast<uint32_t>(nowMs - lastRunningAgentPingMs) < RUNNING_AGENT_PING_PERIOD_MS) {
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

void rosTask() {
  const uint32_t nowMs = millis();
  switch (rosState) {
  case RosState::WAIT_AGENT:
    if (triggerPulseActive()) {
      return;
    }
    if (static_cast<uint32_t>(nowMs - lastAgentPingMs) < AGENT_PING_PERIOD_MS) {
      return;
    }
    lastAgentPingMs = nowMs;
    if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
      rosState = RosState::CREATE_ENTITIES;
    }
    break;

  case RosState::CREATE_ENTITIES:
    if (createEntities()) {
      rosState = RosState::SYNC_TIME;
    } else {
      destroyEntities();
      ++reconnectCount;
      rosState = RosState::WAIT_AGENT;
    }
    break;

  case RosState::SYNC_TIME:
    if (rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS) == RMW_RET_OK &&
        rmw_uros_epoch_synchronized()) {
      epochOffsetNs =
          rmw_uros_epoch_nanos() - static_cast<int64_t>(time_us_64() * 1000ULL);
      timeSynchronized = true;
      ++timeSyncCount;
      setBootStage(BootStage::TIME_SYNC_READY);
      rosState = RosState::RUNNING;
    } else {
      rosState = RosState::DESTROY_ENTITIES;
    }
    break;

  case RosState::RUNNING:
    rosSessionError = false;
    if (!checkRunningAgent(nowMs)) {
      rosState = RosState::DESTROY_ENTITIES;
      break;
    }
    publishHeartbeat(nowMs);
    publishImuIfReady();
    publishRanges();
    publishObstacle(nowMs);
    publishDiagnostics(nowMs);
    setBootStage(BootStage::RUNNING);
    if (rosSessionError) {
      rosState = RosState::DESTROY_ENTITIES;
    }
    break;

  case RosState::DESTROY_ENTITIES:
    destroyEntities();
    ++reconnectCount;
    rosState = RosState::WAIT_AGENT;
    break;
  }
}

void setupUltrasonics() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    sensors[i].health = SensorHealth::INIT;
    pinMode(sensors[i].trig_pin, OUTPUT);
    digitalWrite(sensors[i].trig_pin, LOW);
    pinMode(sensors[i].echo_pin, INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(sensors[0].echo_pin), echoIsrU1,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensors[1].echo_pin), echoIsrU2,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensors[2].echo_pin), echoIsrU3,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensors[3].echo_pin), echoIsrU4,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensors[4].echo_pin), echoIsrU5,
                  CHANGE);
  scanWindowStartUs = time_us_64();
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  initializeBootMarker();
  blinkPreviousBootStage();
  set_microros_transports();
  setBootStage(BootStage::TRANSPORT_CONFIGURED);
  zeroEntities();
  initImuMessage();
  initRangeMessages();
  initDiagnostics();
  setBootStage(BootStage::SENSOR_SHARED_STATE_READY);
  if constexpr (ENABLE_ULTRASONICS) {
    setupUltrasonics();
  } else {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      sensors[i].health = SensorHealth::DISABLED;
    }
  }
  core0ReadyForCore1.store(1U, std::memory_order_release);
  setBootStage(BootStage::CORE1_RELEASED);
}

void setup1() {
  while (core0ReadyForCore1.load(std::memory_order_acquire) == 0U) {
    delay(1);
  }
  if constexpr (ENABLE_IMU) {
    imuState.state = ImuRunState::OFFLINE;
    publishImuSnapshot();
    (void)initializeImu();
    imuCoreReady.store(1U, std::memory_order_release);
  } else {
    imuState.state = ImuRunState::DISABLED;
    publishImuSnapshot();
    imuCoreReady.store(1U, std::memory_order_release);
  }
}

void loop() {
  const uint32_t loopStartUs = micros();
  if constexpr (ENABLE_ULTRASONICS) {
    ultrasonicTask();
    synchronizeGlitchCounters();
    refreshSensorHealth();
    updateScanRate();
  }
  rosTask();
  const uint32_t durationUs = static_cast<uint32_t>(micros() - loopStartUs);
  if (durationUs > loopMaxUs) {
    loopMaxUs = durationUs;
  }
}

void loop1() {
  if constexpr (ENABLE_IMU) {
    imuTask(micros());
  }
}
