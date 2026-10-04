#include <Arduino.h>
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
#ifndef PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
#define PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER 0
#endif
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/executor_handle.h>
#include <rclc/subscription.h>
#include <rmw_microros/ping.h>
#include <rmw_microros/timing.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
#include <geometry_msgs/msg/twist.h>
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
#define PICO_B_MOTOR_CYCLE_HZ 20
#endif
#ifndef PICO_B_ROS_DOMAIN_ID
#define PICO_B_ROS_DOMAIN_ID 42
#endif
#if PICO_B_ROS_DOMAIN_ID < 0 || PICO_B_ROS_DOMAIN_ID > 232
#error "PICO_B_ROS_DOMAIN_ID must be in valid DDS domain range [0..232]"
#endif
#ifndef PICO_B_ENABLE_SERVO
#define PICO_B_ENABLE_SERVO 0
#endif
#ifndef PICO_B_ENABLE_SPI_BRIDGE
#define PICO_B_ENABLE_SPI_BRIDGE 0
#endif
#ifndef PICO_B_ENABLE_MANUAL_OLED_PAGING
#define PICO_B_ENABLE_MANUAL_OLED_PAGING 1
#endif
#ifndef PICO_B_ENABLE_RELAYS
#define PICO_B_ENABLE_RELAYS 0
#endif
#ifndef PICO_B_COAST_TEST
#define PICO_B_COAST_TEST 0
#endif
#if PICO_B_COAST_TEST != 0 && PICO_B_COAST_TEST != 1
#error "PICO_B_COAST_TEST must be 0 (brake-on, production) or 1 (coast, experimental only)"
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
constexpr uint32_t MIN_OLED_BUDGET_US = 1500;
constexpr size_t   OLED_CHUNK_BYTES = 16;
constexpr uint32_t OLED_CHUNK_SAFE_BUDGET_US = 1100;
constexpr uint8_t  OLED_SETUP_CHUNK_COUNT = 1;
constexpr uint16_t OLED_TOTAL_FRAMEBUFFER_BYTES = 128 * ((64 + 7) / 8);
constexpr uint8_t  OLED_DATA_CHUNK_COUNT =
    (OLED_TOTAL_FRAMEBUFFER_BYTES + OLED_CHUNK_BYTES - 1) / OLED_CHUNK_BYTES;
constexpr uint8_t  OLED_TOTAL_CHUNKS_PER_FRAME =
    OLED_SETUP_CHUNK_COUNT + OLED_DATA_CHUNK_COUNT;
static_assert(OLED_CHUNK_BYTES == 16, "B5F production chunk size: 16 bytes per IDLE safe slot");
static_assert(OLED_CHUNK_SAFE_BUDGET_US >= 800 && OLED_CHUNK_SAFE_BUDGET_US <= 1500,
              "B5F safe budget: 800..1500 us (typical 16-byte transfer ~530-610 us)");
static_assert(OLED_DATA_CHUNK_COUNT == 64, "128x64 FB 1024 bytes / 16 bytes = 64 chunks");
static_assert(OLED_TOTAL_CHUNKS_PER_FRAME == 65, "1 setup + 64 data chunks = 65 per frame");
constexpr uint32_t OLED_PREP_SAFE_BUDGET_US = 1600;
static_assert(OLED_PREP_SAFE_BUDGET_US >= 1200 && OLED_PREP_SAFE_BUDGET_US <= 2500,
              "B5G frame prep budget: clearDisplay + drawXxxPage typical 800..1300 us <= 1600 (1.25x)");
constexpr uint32_t CMD_VEL_FRESH_TIMEOUT_MS = 250;
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
constexpr uint32_t MOTOR_STOP_SETTLE_MIN_MS = 250;
constexpr uint32_t MOTOR_STOP_SETTLE_MAX_MS = 1000;
constexpr uint8_t  MOTOR_STOP_STABLE_CYCLES = 3;
constexpr uint32_t MOTOR_RATE_WINDOW_MS = 1000;
constexpr uint32_t MOTOR_TEMPERATURE_QUERY_PERIOD_MS = 1000;
constexpr uint32_t MOTOR_TEMPERATURE_PHASE_STEP_MS =
    MOTOR_TEMPERATURE_QUERY_PERIOD_MS / MOTOR_COUNT;
constexpr uint8_t MOTOR_MAX_CONSECUTIVE_ERRORS = 3;

constexpr int16_t  MOTOR_MOTION_TEST_MAX_RPM   = 20;
constexpr int16_t  MOTOR_PIVOT_TEST_MAX_RPM    = 10;
constexpr int16_t  MOTION_SLEW_LIMIT_PER_CYCLE = 2;
constexpr uint32_t MOTION_NEUTRAL_ARMING_MS    = 500;
constexpr uint8_t  SBUS_THROTTLE_CH            = 1;
constexpr uint8_t  SBUS_STEERING_CH            = 0;
constexpr float    MOTION_DEADBAND_NORM        = 0.03f;
constexpr uint16_t SBUS_THROTTLE_FORWARD_RAW   = 282;
constexpr uint16_t SBUS_THROTTLE_CENTER_RAW    = 1002;
constexpr uint16_t SBUS_THROTTLE_REVERSE_RAW   = 1722;

// Production DDSM115 drive hard-cap (absolute maximum magnitude any mode may
// request in a single motor RPM target).  No mode cap may exceed this value.
constexpr int16_t  PICO_B_PRODUCTION_MAX_DRIVE_RPM = 80;

static_assert(SBUS_THROTTLE_FORWARD_RAW < SBUS_THROTTLE_CENTER_RAW,
              "Throttle calibration: forward raw must be below center (inverted physical channel)");
static_assert(SBUS_THROTTLE_CENTER_RAW < SBUS_THROTTLE_REVERSE_RAW,
              "Throttle calibration: center raw must be below reverse raw");
constexpr int8_t   MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = {
    +1,
    +1,
    -1,
    -1
};

static_assert(MOTOR_DIRECTION_SIGN[0] == +1 && MOTOR_DIRECTION_SIGN[1] == +1 &&
              MOTOR_DIRECTION_SIGN[2] == -1 && MOTOR_DIRECTION_SIGN[3] == -1,
              "Motor direction convention: left side FL/RL positive-forward; right side FR/RR negative-forward (physical wiring verified)");
static_assert(MOTOR_MOTION_TEST_MAX_RPM > 0, "Motion unlock: positive test speed required");
static_assert(MOTOR_PIVOT_TEST_MAX_RPM > 0, "Motion unlock: positive pivot test speed required");
static_assert(PICO_B_PRODUCTION_MAX_DRIVE_RPM > 0, "Production cap: strictly positive hard RPM bound required");
static_assert(PICO_B_PRODUCTION_MAX_DRIVE_RPM == 80,
              "Production cap: user-specified absolute drive RPM ceiling is 80 (per validated hardware)");
static_assert(MOTION_SLEW_LIMIT_PER_CYCLE >= 1,
              "Motion unlock: slew limiter must allow at least 1 RPM per cycle");
static_assert(MOTION_SLEW_LIMIT_PER_CYCLE <= PICO_B_PRODUCTION_MAX_DRIVE_RPM,
              "Motion unlock: slew limiter cannot exceed absolute drive RPM cap");
static_assert(MOTION_NEUTRAL_ARMING_MS >= 100,
              "Motion unlock: neutral arming must take at least 100 ms");

#if PICO_B_COAST_TEST
constexpr uint8_t MOTOR_BRAKE_ACTIVE_BYTE = 0x00;
#else
constexpr uint8_t MOTOR_BRAKE_ACTIVE_BYTE = 0xFF;
#endif
#if PICO_B_COAST_TEST
static_assert(MOTOR_BRAKE_ACTIVE_BYTE == 0x00,
              "PICO_B_COAST_TEST variant requires DDSM115 brake byte 0x00 (coast, experimental only)");
#else
static_assert(MOTOR_BRAKE_ACTIVE_BYTE == 0xFF,
              "B3A requires active DDSM115 brake");
#endif
static_assert(RS485_SPEED_FIRST_ATTEMPT_TOTAL_MS == 15,
              "Speed first-attempt window must remain 15 ms");
static_assert(MOTOR_TEMPERATURE_PHASE_STEP_MS == 250,
              "Temperature queries must remain phase-spaced by 250 ms");
static_assert(PICO_B_MOTOR_CYCLE_HZ == 10 || PICO_B_MOTOR_CYCLE_HZ == 15 ||
                  PICO_B_MOTOR_CYCLE_HZ == 20,
              "PICO_B_MOTOR_CYCLE_HZ must be 10, 15, or 20");

// =============================================================================
// CH6 3-POSITION DRIVE MODE SELECTOR
// SBUS CH6 (1-based) = channels[5] zero-based.
// Physical switch positions (live measured):
//   LOW    ~ 282   -> FAST      (raw <= DRIVE_MODE_FAST_MAX)
//   CENTER ~ 1002  -> NORMAL    (raw between FAST_MAX+1 and AUTO_MIN-1)
//   HIGH   ~ 1722  -> AUTONOMOUS (raw >= DRIVE_MODE_AUTO_MIN)
// Thresholds mirror OLED paging zones (CH3 uses same triplet 700/900-1100/1300).
// =============================================================================
enum class DriveMode : uint8_t {
  FAST,
  NORMAL,
  AUTONOMOUS,
};

constexpr uint8_t  SBUS_DRIVE_MODE_CH      = 5;   // zero-based = CH6 1-based
constexpr uint16_t DRIVE_MODE_FAST_MAX     = 700;  // raw <=700: FAST  (LOW switch)
constexpr uint16_t DRIVE_MODE_AUTO_MIN     = 1300; // raw >=1300: AUTONOMOUS (HIGH switch)

// Mode RPM caps.
// Global production hard limit: NO MODE may exceed PICO_B_PRODUCTION_MAX_DRIVE_RPM = 80
// (per-mode static_asserts below enforce this for drive; pivot is always 8/10 ≤ 80).
//
// FAST mode:     user validated top speed (CH6 LOW)    → drive = 80 / pivot = 10
// NORMAL mode:   production safe default (CH6 CENTER) → drive = 40 / pivot =  8
// AUTONOMOUS mode: conservative ROS cmd_vel (CH6 HIGH)→ drive = 20 / pivot = 10
constexpr int16_t FAST_MAX_DRIVE_RPM       = 80;
constexpr int16_t NORMAL_MAX_DRIVE_RPM     = 40;
constexpr int16_t FAST_MAX_PIVOT_RPM       = 10;
constexpr int16_t NORMAL_MAX_PIVOT_RPM     =  8;
constexpr int16_t AUTONOMOUS_MAX_DRIVE_RPM = 20;
constexpr int16_t AUTONOMOUS_MAX_PIVOT_RPM = 10;

// Per-mode compile-time invariants: each mode's caps are bounded and consistent.
static_assert(FAST_MAX_DRIVE_RPM       > 0 && FAST_MAX_DRIVE_RPM       <= PICO_B_PRODUCTION_MAX_DRIVE_RPM);
static_assert(NORMAL_MAX_DRIVE_RPM     > 0 && NORMAL_MAX_DRIVE_RPM     <= PICO_B_PRODUCTION_MAX_DRIVE_RPM);
static_assert(AUTONOMOUS_MAX_DRIVE_RPM > 0 && AUTONOMOUS_MAX_DRIVE_RPM <= PICO_B_PRODUCTION_MAX_DRIVE_RPM);
static_assert(FAST_MAX_PIVOT_RPM       > 0 && FAST_MAX_PIVOT_RPM       <= FAST_MAX_DRIVE_RPM);
static_assert(NORMAL_MAX_PIVOT_RPM     > 0 && NORMAL_MAX_PIVOT_RPM     <= NORMAL_MAX_DRIVE_RPM);
static_assert(AUTONOMOUS_MAX_PIVOT_RPM > 0 && AUTONOMOUS_MAX_PIVOT_RPM <= AUTONOMOUS_MAX_DRIVE_RPM);
// Final global invariant: the single largest possible drive magnitude (FAST 80)
// matches the absolute production cap exactly.
static_assert(FAST_MAX_DRIVE_RPM == PICO_B_PRODUCTION_MAX_DRIVE_RPM,
              "FAST mode must saturate the absolute production drive RPM hard-cap");

// Differential-drive kinematics for ROS cmd_vel -> wheel RPM conversion.
// Matches lessien_description SSoT geometry.yaml exactly.
constexpr float WHEEL_RADIUS_M             = 0.05035f;  // SSoT
constexpr float TRACK_WIDTH_M              = 0.145f;    // SSoT = wheel_separation

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

// --- MERKEZİ CAMERA SERVO KONFIGÜRASYONU (RDS3235) ---
// ! ÖNEMLİ: Aşağıdaki yazılım limitleri KULLANICI tarafından (2026-10-01)
// ! mekanik ölçümle onaylanmıştır:
// !   MIN    = 35 deg
// !   CENTER = 90 deg
// !   FAILSAFE / HOME = 90 deg
// !   MAX    = 145 deg
// !
// ! Pulse limitleri, mevcut KALİBRE base 3-nokta map üzerinden TÜRETİLMİŞTİR
// ! (sabit kalibrasyon: 0°→550us, 90°→1570us, 180°→2450us,
// !  cameraServoAngleToPulseUs() piecewise-linear + 0.5 round):
// !   35°  = 550 + (35/90)*(1570-550) + 0.5  =  947 us
// !   90°  = 1570 us                           (sabit)
// !   145° = 1570 + ((145-90)/90)*(2450-1570) + 0.5 = 2108 us
// !
// ! Absolute hardware safety limiti (550..2450 us) korunur; ama yazılım aralığı
// ! 947..2108 us ile sınırlıdır.
// ! RDS3235 fiziksel pozisyon feedback pininin bulunmaması nedeniyle
// ! currentAngleTenthsDeg ve currentPulseUs YAZILIMSAL TAHMİNDİR;
// ! gerçek mekanik pozisyon yük, stall veya dışarıdan itme altında sapabilir.
constexpr int16_t  CAM_SERVO_MIN_ANGLE_DEG     = 35;
constexpr int16_t  CAM_SERVO_CENTER_ANGLE_DEG  = 90;
constexpr int16_t  CAM_SERVO_MAX_ANGLE_DEG     = 145;
constexpr uint16_t CAM_SERVO_HOME_PULSE_US     = 1570;
constexpr uint16_t CAM_SERVO_FAILSAFE_PULSE_US = 1570;
constexpr uint16_t CAM_SERVO_MIN_PULSE_US      =  947;
constexpr uint16_t CAM_SERVO_MAX_PULSE_US      = 2108;
constexpr uint16_t CAM_SERVO_ABSOLUTE_MIN_US   = SERVO_ABSOLUTE_MIN_US;
constexpr uint16_t CAM_SERVO_ABSOLUTE_MAX_US   = SERVO_ABSOLUTE_MAX_US;
constexpr uint32_t CAM_SERVO_UPDATE_PERIOD_MS  = SERVO_UPDATE_PERIOD_MS;
constexpr uint16_t CAM_SERVO_MAX_SLEW_DEG_P_S  = SERVO_MAX_SLEW_DEG_PER_S;
constexpr int16_t  CAM_SERVO_SLEW_TENTHS_PER_UPDATE =
    SERVO_SLEW_TENTHS_DEG_PER_UPDATE;

static_assert(CAM_SERVO_ABSOLUTE_MIN_US <= CAM_SERVO_MIN_PULSE_US,
              "CAM_SERVO: MIN_PULSE_US (947) must be >= driver ABSOLUTE_MIN_US (550)");
static_assert(CAM_SERVO_MAX_PULSE_US <= CAM_SERVO_ABSOLUTE_MAX_US,
              "CAM_SERVO: MAX_PULSE_US (2108) must be <= driver ABSOLUTE_MAX_US (2450)");
static_assert(CAM_SERVO_HOME_PULSE_US == 1570 && CAM_SERVO_HOME_PULSE_US == SERVO_CENTER_US,
              "CAM_SERVO: home pulse must remain 90deg = 1570 us");
static_assert(CAM_SERVO_FAILSAFE_PULSE_US == 1570,
              "CAM_SERVO: failsafe pulse must remain 90deg = 1570 us");
static_assert(CAM_SERVO_FAILSAFE_PULSE_US >= CAM_SERVO_MIN_PULSE_US &&
                  CAM_SERVO_FAILSAFE_PULSE_US <= CAM_SERVO_MAX_PULSE_US,
              "CAM_SERVO: failsafe pulse (1570) is outside the software range (947..2108)");
static_assert(CAM_SERVO_HOME_PULSE_US >= CAM_SERVO_MIN_PULSE_US &&
                  CAM_SERVO_HOME_PULSE_US <= CAM_SERVO_MAX_PULSE_US,
              "CAM_SERVO: home pulse (1570) is outside the software range (947..2108)");
static_assert(CAM_SERVO_MIN_ANGLE_DEG == 35 &&
                  CAM_SERVO_CENTER_ANGLE_DEG == 90 &&
                  CAM_SERVO_MAX_ANGLE_DEG == 145,
              "CAM_SERVO: software angle range must be 35..90..145 deg");
static_assert(CAM_SERVO_MIN_PULSE_US == 947,
              "CAM_SERVO: MIN pulse must be the derived value for 35deg");
static_assert(CAM_SERVO_MAX_PULSE_US == 2108,
              "CAM_SERVO: MAX pulse must be the derived value for 145deg");

constexpr char NODE_NAME[] = "pico_b_control_node";
constexpr char HEARTBEAT_TOPIC[] = "/pico_b/heartbeat";
constexpr char SBUS_TOPIC[] = "/pico_b/sbus/channels";
constexpr char BATTERY_TOPIC[] = "/pico_b/battery/voltage";
constexpr char MOTORS_RPM_TOPIC[] = "/pico_b/motors/rpm";
constexpr char DIAGNOSTICS_TOPIC[] = "/diagnostics";
constexpr char CMD_VEL_TOPIC[] = "/cmd_vel";
constexpr char CMD_VEL_DIAG_NAME[] = "pico_b/cmd_vel";
constexpr size_t CMD_VEL_EXECUTOR_HANDLE_COUNT = 1;
#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
constexpr char EKF_DISTANCE_TOPIC[] = "/lessien/ekf_distance_m";
constexpr char EKF_DISTANCE_DIAG_NAME[] = "pico_b/ekf_distance";
constexpr size_t EKF_DISTANCE_EXECUTOR_HANDLE_COUNT = 1;
constexpr uint32_t EKF_DISTANCE_STALE_TIMEOUT_MS = 1000;
#endif
constexpr size_t SBUS_CHANNEL_COUNT = 16;
constexpr size_t DIAG_STATUS_CAPACITY = 1;
constexpr size_t DIAG_VALUE_CAPACITY = 15;
constexpr size_t DIAG_VALUE_TEXT_CAPACITY = 24;
static_assert(DIAG_VALUE_CAPACITY >= 15, "B5G: 6 legacy + 6 chunk + 3 prep/error = 15 values min");

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
  DRIVE_MODE,
  EKF_DISTANCE,
};

enum class OledPage : uint8_t {
  BATTERY,
  SYSTEM,
  MOTORS,
  DRIVE_MODE,
  EKF_TRIP,
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

enum class MotorStopState : uint8_t {
  MOVING = 0,
  ZERO_SETTLE = 1,
  BRAKE_HOLD = 2,
  FAILSAFE_BRAKE = 3,
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

// FİZİKSEL GERİ BİLDİRİM SINIRLAMASI:
// RDS3235 camera servoda position feedback pini kullanılmamaktadır.
// currentAngleTenthsDeg ve currentPulseUs alanları SADECE yazılımsal tahmindir
// ve komut edilen output + kalibre edilmiş angle->pulse haritalamasından
// türetilir. Mekanik yük, stall veya dışarıdan itme durumunda yazılımsal
// açı, reel fiziksel açıdan SAPABİLİR. Limit clamp ve smooth hareket
// MEVCUT YAZILIMSAL DURUM üzerinden çalışır; reel pozisyonu garanti etmez.
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

#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
struct EkfDistanceStatus {
  float distanceM;
  uint32_t receiveCount;
  uint32_t rejectCount;
  uint32_t lastReceiveMs;
  bool distanceReceived;
  bool distanceFinite;
};
#endif

constexpr MotorConfig MOTOR_CONFIGS[MOTOR_COUNT] = {
  {1, "FL"},
  {2, "RL"},
  {3, "FR"},
  {4, "RR"},
};
static_assert(MOTOR_COUNT == 4, "Exactly four DDSM115 motors are required");
static_assert(MOTOR_SLOT_PERIOD_US == 25000UL ||
                  MOTOR_SLOT_PERIOD_US == 16666UL ||
                  MOTOR_SLOT_PERIOD_US == 12500UL,
              "Unexpected motor slot period");
#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER
#if PICO_B_COAST_TEST
static_assert(MOTOR_BRAKE_ACTIVE_BYTE == 0x00,
              "B5A (coast test build): brake byte 0x00; motion still disabled by RPM=0");
#else
static_assert(MOTOR_BRAKE_ACTIVE_BYTE == 0xFF,
              "B5A: cmd_vel subscriber must NOT enable motor motion; brake byte 0xFF");
#endif
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
#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
rclc_executor_t ekfDistanceExecutor;
rcl_subscription_t ekfDistanceSubscriber;
std_msgs__msg__Float32 ekfDistanceMsg;
bool ekfDistanceSubscriberCreated = false;
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
#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
EkfDistanceStatus ekfDistanceStatus = {};
#endif

RosState rosState = RosState::WAIT_AGENT;
DiagnosticPage diagnosticPage = DiagnosticPage::SBUS;
OledPage oledPage = OledPage::BATTERY;

enum class OledTransferState : uint8_t {
    IDLE = 0,
    TRANSFER_ACTIVE = 1,
};
OledTransferState oledTransferState = OledTransferState::IDLE;
bool oledRedrawRequested = false;
bool oledFramePrepPending = false;
uint8_t oledChunkIndex = 0;
uint32_t oledFrameCumulativeUs = 0;
uint32_t currentMotorSlotEpoch = 0;
uint32_t oledLastServicedSlotEpoch = 0;
uint32_t oledChunkLastUs = 0;
uint32_t oledChunkWindowMaxUs = 0;
uint32_t oledChunkSkippedCount = 0;
uint32_t oledChunkTxErrorCount = 0;
uint32_t oledFrameTransferCount = 0;
uint32_t oledPrepLastUs = 0;
uint32_t oledPrepWindowMaxUs = 0;

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
bool oledSplashShown = false;
uint32_t oledBudgetDroppedFrameCount = 0;
uint32_t oledPageChangeCount = 0;
uint32_t oledIgnoredInputCount = 0;
uint32_t relayFailsafeCount = 0;
uint8_t relayInhibitCode = RELAY_INHIBIT_DISABLED;
bool relayOutputSafe = false;
uint32_t lastRelayTaskMs = 0;
uint32_t cameraServoFailsafeCount = 0;
MotorStopState motorStopState = MotorStopState::ZERO_SETTLE;
uint32_t motorStopSettleStartMs = 0;
uint8_t  motorStopStableCycles = 0;
uint32_t motorStopSettleTimeoutCount = 0;
uint8_t  motorStopPrevActiveIndex = 0xFF;

int16_t  requestedMotorRpm[MOTOR_COUNT] = {0, 0, 0, 0};
bool     motionArmed = false;
uint32_t motionNeutralArmingStartMs = 0;
float    motionLastThrottleNorm = 0.0f;
float    motionLastSteeringNorm = 0.0f;
int16_t  motionSlewRequestedRpm[MOTOR_COUNT] = {0, 0, 0, 0};
uint8_t  motionLastCycleIndex = 0xFF;

// CH6 selected drive mode state + last raw sample for diagnostics.
DriveMode selectedDriveMode     = DriveMode::NORMAL;
uint16_t  lastDriveModeCh6Raw   = SBUS_CENTER;
// Last command source actually used to compute requested RPM (diagnostics).
bool      lastMotionSourceWasSbus = true;

// DriveMode -> human-readable label (OLED page + diagnostics).
inline const char *driveModeText(DriveMode mode) {
  switch (mode) {
    case DriveMode::FAST:       return "FAST";
    case DriveMode::NORMAL:     return "NORMAL";
    case DriveMode::AUTONOMOUS: return "AUTONOMOUS";
  }
  return "NORMAL";
}

// DriveMode -> 3-char OLED corner marker.  Does NOT exceed 3 chars so it fits
// in the top-right corner of every OLED page without clobbering layout.
inline const char *driveModeMarker(DriveMode mode) {
  switch (mode) {
    case DriveMode::FAST:       return "[F]";
    case DriveMode::NORMAL:     return "[N]";
    case DriveMode::AUTONOMOUS: return "[A]";
  }
  return "[N]";
}

// True when the last motion command source was SBUS joystick (NORMAL/FAST).
// False when source was ROS /cmd_vel (AUTONOMOUS).
inline bool motionSourceIsSbus() {
  return selectedDriveMode != DriveMode::AUTONOMOUS;
}

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

#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
void invalidateEkfDistanceSnapshot() {
  ekfDistanceStatus.distanceM = 0.0f;
  ekfDistanceStatus.distanceReceived = false;
  ekfDistanceStatus.distanceFinite = false;
}

void ekfDistanceCallback(const void * msgin) {
  const auto *floatMsg = static_cast<const std_msgs__msg__Float32 *>(msgin);
  if (floatMsg == nullptr) {
    ++ekfDistanceStatus.rejectCount;
    return;
  }
  const float d = floatMsg->data;
  const bool finite = isfinite(d);
  if (!finite) {
    ++ekfDistanceStatus.rejectCount;
    ekfDistanceStatus.distanceFinite = false;
    return;
  }
  const uint32_t nowMs = millis();
  ekfDistanceStatus.distanceM = d;
  ekfDistanceStatus.distanceFinite = true;
  ekfDistanceStatus.distanceReceived = true;
  ekfDistanceStatus.lastReceiveMs = nowMs;
  ++ekfDistanceStatus.receiveCount;
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
#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
  ekfDistanceSubscriber = rcl_get_zero_initialized_subscription();
  ekfDistanceExecutor = {};
  ekfDistanceSubscriberCreated = false;
  ekfDistanceMsg = {};
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
#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
  if (ekfDistanceSubscriber.impl != nullptr) {
    const rcl_ret_t retSub = rcl_subscription_fini(&ekfDistanceSubscriber, &node);
    (void)retSub;
  }
  ekfDistanceSubscriber = rcl_get_zero_initialized_subscription();
  if (ekfDistanceSubscriberCreated) {
    const rcl_ret_t retExec = rclc_executor_fini(&ekfDistanceExecutor);
    (void)retExec;
  }
  ekfDistanceExecutor = {};
  ekfDistanceSubscriberCreated = false;
  invalidateEkfDistanceSnapshot();
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

#if PICO_B_ENABLE_MOTORS
bool motorAllowsSlowTasks();
#endif

const char *oledPageName() {
  switch (oledPage) {
    case OledPage::BATTERY:
      return "BATTERY";
    case OledPage::SYSTEM:
      return "SYSTEM";
    case OledPage::MOTORS:
      return "MOTORS";
    case OledPage::DRIVE_MODE:
      return "DRIVE_MODE";
    case OledPage::EKF_TRIP:
      return "TRIP";
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
  if (oledTransferState == OledTransferState::TRANSFER_ACTIVE) {
    oledRedrawRequested = true;
    return;
  }
  oledRedrawRequested = true;
  oledFramePending = true;
}

void serviceOledManualPaging(uint32_t nowMs) {
#if PICO_B_ENABLE_OLED && PICO_B_ENABLE_MANUAL_OLED_PAGING
#if PICO_B_ENABLE_MOTORS
  if (!motorAllowsSlowTasks()) {
    ++oledIgnoredInputCount;
    return;
  }
#endif
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

float normalizeThrottleChannel(uint16_t raw) {
  const int32_t delta =
      static_cast<int32_t>(raw) - static_cast<int32_t>(SBUS_THROTTLE_CENTER_RAW);

  if (delta >= -static_cast<int32_t>(SBUS_DEADBAND) &&
      delta <=  static_cast<int32_t>(SBUS_DEADBAND)) {
    return 0.0f;
  }

  if (raw < SBUS_THROTTLE_CENTER_RAW) {
    const float span =
        static_cast<float>(SBUS_THROTTLE_CENTER_RAW - SBUS_THROTTLE_FORWARD_RAW);
    float value =
        static_cast<float>(SBUS_THROTTLE_CENTER_RAW - raw) / span;
    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;
    return +value;
  }

  const float span =
      static_cast<float>(SBUS_THROTTLE_REVERSE_RAW - SBUS_THROTTLE_CENTER_RAW);
  float value =
      static_cast<float>(raw - SBUS_THROTTLE_CENTER_RAW) / span;
  if (value < 0.0f) value = 0.0f;
  if (value > 1.0f) value = 1.0f;
  return -value;
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
  const float relativeAngle = ch4Normalized * 90.0f;
  const int16_t roundedAngle =
      static_cast<int16_t>(relativeAngle >= 0.0f ? relativeAngle + 0.5f
                                                 : relativeAngle - 0.5f);
  return constrain(roundedAngle, -90, 90);
}

uint16_t cameraServoAngleToPulseUs(float angleDeg) {
  if (!isfinite(angleDeg)) {
    angleDeg = CAM_SERVO_CENTER_ANGLE_DEG;
  }
  const float limitedAngleDeg =
      constrain(angleDeg, static_cast<float>(CAM_SERVO_MIN_ANGLE_DEG),
                static_cast<float>(CAM_SERVO_MAX_ANGLE_DEG));
  int32_t pulseUs;
  if (limitedAngleDeg <= CAM_SERVO_CENTER_ANGLE_DEG) {
    const float fraction =
        (limitedAngleDeg - CAM_SERVO_MIN_ANGLE_DEG) /
        (CAM_SERVO_CENTER_ANGLE_DEG - CAM_SERVO_MIN_ANGLE_DEG);
    pulseUs = static_cast<int32_t>(
        CAM_SERVO_MIN_PULSE_US +
        fraction * (CAM_SERVO_HOME_PULSE_US - CAM_SERVO_MIN_PULSE_US) + 0.5f);
  } else {
    const float fraction =
        (limitedAngleDeg - CAM_SERVO_CENTER_ANGLE_DEG) /
        (CAM_SERVO_MAX_ANGLE_DEG - CAM_SERVO_CENTER_ANGLE_DEG);
    pulseUs = static_cast<int32_t>(
        CAM_SERVO_HOME_PULSE_US +
        fraction * (CAM_SERVO_MAX_PULSE_US - CAM_SERVO_HOME_PULSE_US) + 0.5f);
  }
  // Katman 1 clamp: yazılım min/max pulse
  pulseUs = constrain(pulseUs,
                      static_cast<int32_t>(CAM_SERVO_MIN_PULSE_US),
                      static_cast<int32_t>(CAM_SERVO_MAX_PULSE_US));
  // Katman 2 clamp: sürücü ABSOLUTE min/max pulse
  pulseUs = constrain(pulseUs,
                      static_cast<int32_t>(CAM_SERVO_ABSOLUTE_MIN_US),
                      static_cast<int32_t>(CAM_SERVO_ABSOLUTE_MAX_US));
  return static_cast<uint16_t>(pulseUs);
}

uint16_t cameraServoPulseFromAngleTenths(int16_t angleTenthsDeg) {
  return cameraServoAngleToPulseUs(
      static_cast<float>(angleTenthsDeg) / 10.0f);
}

void initializeCameraServo(uint32_t nowMs) {
  cameraServoStatus.attached = false;
  cameraServoStatus.inputHealthy = false;
  cameraServoStatus.targetRelativeAngleDeg = 0;
  cameraServoStatus.targetAbsoluteAngleDeg = CAM_SERVO_CENTER_ANGLE_DEG;
  cameraServoStatus.currentAngleTenthsDeg = CAM_SERVO_CENTER_ANGLE_DEG * 10;
  cameraServoStatus.targetPulseUs = CAM_SERVO_HOME_PULSE_US;
  cameraServoStatus.currentPulseUs = CAM_SERVO_HOME_PULSE_US;
  cameraServoStatus.lastUpdateMs = nowMs;
  cameraServoStatus.seenValidFrameCount = sbusStatus.validFrameCount;
  cameraServoStatus.seenInvalidFrameCount = sbusStatus.invalidFrameCount;
  cameraServoStatus.sbusFrameValid = false;
  cameraServoFailsafeCount = 0;
#if PICO_B_ENABLE_CAMERA_SERVO
  cameraServoStatus.attached =
      cameraServoOutput.attach(CAMERA_SERVO_SIGNAL_PIN,
                               CAM_SERVO_ABSOLUTE_MIN_US,
                               CAM_SERVO_ABSOLUTE_MAX_US,
                               CAM_SERVO_HOME_PULSE_US) >= 0;
  if (cameraServoStatus.attached) {
    // İLK YAZMA: aniden sıçramayı önlemek için HOME pulse, iki katmanlı clamp
    uint16_t safeHomeUs = CAM_SERVO_HOME_PULSE_US;
    safeHomeUs = static_cast<uint16_t>(constrain(
        static_cast<int32_t>(safeHomeUs),
        static_cast<int32_t>(CAM_SERVO_MIN_PULSE_US),
        static_cast<int32_t>(CAM_SERVO_MAX_PULSE_US)));
    safeHomeUs = static_cast<uint16_t>(constrain(
        static_cast<int32_t>(safeHomeUs),
        static_cast<int32_t>(CAM_SERVO_ABSOLUTE_MIN_US),
        static_cast<int32_t>(CAM_SERVO_ABSOLUTE_MAX_US)));
    cameraServoOutput.writeMicroseconds(safeHomeUs);
    cameraServoStatus.currentPulseUs = safeHomeUs;
    cameraServoStatus.targetPulseUs = safeHomeUs;
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

  // Hedef belirle: input sağlıklıysa SBUS CH4, aksi halde FAILSAFE pulse'a
  // karşılık gelen açı (CENTER). Bu bölümde sadece TARGET hesaplanır; output
  // periyodik update kısıtlaması sonrası smooth hareket ile yazılır.
  const int16_t relativeDeg =
      inputHealthy ? cameraServoTargetRelativeAngleFromCh4() : 0;
  int16_t targetAngleDeg =
      CAM_SERVO_CENTER_ANGLE_DEG + relativeDeg;
  targetAngleDeg = constrain(targetAngleDeg,
                             CAM_SERVO_MIN_ANGLE_DEG,
                             CAM_SERVO_MAX_ANGLE_DEG);
  cameraServoStatus.targetRelativeAngleDeg = relativeDeg;
  cameraServoStatus.targetAbsoluteAngleDeg = targetAngleDeg;

  if (static_cast<uint32_t>(nowMs - cameraServoStatus.lastUpdateMs) <
      CAM_SERVO_UPDATE_PERIOD_MS) {
    return;
  }
  cameraServoStatus.lastUpdateMs = nowMs;

  // NON-BLOCKING smooth hareket: standart constrain(delta, -max, +max) kalıbı
  const int16_t targetTenths = targetAngleDeg * 10;
  int16_t current = cameraServoStatus.currentAngleTenthsDeg;
  const int16_t error = targetTenths - current;
  const int16_t maxStep = CAM_SERVO_SLEW_TENTHS_PER_UPDATE;
  const int16_t delta = constrain(error, -maxStep, maxStep);
  current += delta;
  // Angle katman clamp: tekrar
  current = constrain(current,
                      CAM_SERVO_MIN_ANGLE_DEG * 10,
                      CAM_SERVO_MAX_ANGLE_DEG * 10);
  cameraServoStatus.currentAngleTenthsDeg = current;

  // Pulse üret + İKİ KATMANLI CLAMP (fonksiyon içinde yapılıyor, yine de
  // kesin olarak harici constrain konulur.)
  uint16_t pulseUs = cameraServoPulseFromAngleTenths(current);
  pulseUs = static_cast<uint16_t>(constrain(
      static_cast<int32_t>(pulseUs),
      static_cast<int32_t>(CAM_SERVO_MIN_PULSE_US),
      static_cast<int32_t>(CAM_SERVO_MAX_PULSE_US)));
  pulseUs = static_cast<uint16_t>(constrain(
      static_cast<int32_t>(pulseUs),
      static_cast<int32_t>(CAM_SERVO_ABSOLUTE_MIN_US),
      static_cast<int32_t>(CAM_SERVO_ABSOLUTE_MAX_US)));
  cameraServoStatus.targetPulseUs = pulseUs;
  cameraServoStatus.currentPulseUs = pulseUs;
  cameraServoOutput.writeMicroseconds(pulseUs);
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

static inline uint8_t getRuntimeMotorBrakeByte() {
#if PICO_B_COAST_TEST
  return 0x00;
#else
  switch (motorStopState) {
    case MotorStopState::MOVING:        return 0x00;
    case MotorStopState::ZERO_SETTLE:   return 0x00;
    case MotorStopState::BRAKE_HOLD:    return 0xFF;
    case MotorStopState::FAILSAFE_BRAKE:return 0xFF;
  }
  return 0xFF;
#endif
}

static inline int16_t getRuntimeMotorTargetRpm(uint8_t motorIndex) {
#if PICO_B_COAST_TEST
  return requestedMotorRpm[motorIndex];
#else
  switch (motorStopState) {
    case MotorStopState::MOVING:
      return requestedMotorRpm[motorIndex];
    case MotorStopState::ZERO_SETTLE:
    case MotorStopState::BRAKE_HOLD:
    case MotorStopState::FAILSAFE_BRAKE:
      return 0;
  }
  return 0;
#endif
}

static inline bool motorHasAnyNonzeroTarget() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (requestedMotorRpm[i] != 0) return true;
  }
  return false;
}

static inline bool failsafeStopActive();

static inline bool motorMotionPermitted(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  if (!sbusRealHealthyFrame(nowMs)) return false;
  if (failsafeStopActive()) return false;
  return true;
#else
  (void)nowMs;
  return false;
#endif
}

static inline void updateMotionCommandAndArming(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS && PICO_B_ENABLE_SBUS
  const bool permit = motorMotionPermitted(nowMs);
  // -----------------------------------------------------------------
  // STEP 1: DECODE CH6 DRIVE MODE  (only when SBUS frame is healthy)
  // Priority:  FAILSAFE/SYSTEM STOP  >  DRIVE MODE  >  MAGNITUDES
  // Fail-safe (permit/failsafe checks below) still overrides mode.
  // -----------------------------------------------------------------
  const bool sbusOk = sbusRealHealthyFrame(nowMs);
  uint16_t ch6Raw = SBUS_CENTER;
  if (sbusOk) {
    ch6Raw = sbusStatus.channels[SBUS_DRIVE_MODE_CH];
  }
  lastDriveModeCh6Raw = ch6Raw;
  if (sbusOk) {
    if (ch6Raw <= DRIVE_MODE_FAST_MAX) {
      selectedDriveMode = DriveMode::FAST;
    } else if (ch6Raw >= DRIVE_MODE_AUTO_MIN) {
      selectedDriveMode = DriveMode::AUTONOMOUS;
    } else {
      selectedDriveMode = DriveMode::NORMAL;
    }
  }
  // else: keep last valid drive mode; failsafe/safety gates below stop motion

  // -----------------------------------------------------------------
  // STEP 2: SOURCE SELECTION  (SBUS vs ROS cmd_vel)
  // -----------------------------------------------------------------
  float thrNorm = 0.0f;  // normalized throttle: forward>=0 (+1 full fwd), reverse<=0 (-1 full rev)
  float strNorm = 0.0f;  // normalized steering: left>0, right<0, range [-1, +1]
  int16_t capDrive = NORMAL_MAX_DRIVE_RPM;
  int16_t capPivot = NORMAL_MAX_PIVOT_RPM;
  bool sourceSbus = true;

  switch (selectedDriveMode) {
    case DriveMode::FAST: {
      sourceSbus = true;
      capDrive = FAST_MAX_DRIVE_RPM;
      capPivot = FAST_MAX_PIVOT_RPM;
      const float thrRaw = normalizeThrottleChannel(sbusStatus.channels[SBUS_THROTTLE_CH]);
      const float strRaw = normalizeSbusChannel(sbusStatus.channels[SBUS_STEERING_CH]);
      thrNorm = (fabsf(thrRaw) <= MOTION_DEADBAND_NORM) ? 0.0f : thrRaw;
      strNorm = (fabsf(strRaw) <= MOTION_DEADBAND_NORM) ? 0.0f : strRaw;
    } break;

    case DriveMode::NORMAL: {
      sourceSbus = true;
      capDrive = NORMAL_MAX_DRIVE_RPM;
      capPivot = NORMAL_MAX_PIVOT_RPM;
      const float thrRaw = normalizeThrottleChannel(sbusStatus.channels[SBUS_THROTTLE_CH]);
      const float strRaw = normalizeSbusChannel(sbusStatus.channels[SBUS_STEERING_CH]);
      thrNorm = (fabsf(thrRaw) <= MOTION_DEADBAND_NORM) ? 0.0f : thrRaw;
      strNorm = (fabsf(strRaw) <= MOTION_DEADBAND_NORM) ? 0.0f : strRaw;
    } break;

    case DriveMode::AUTONOMOUS: {
      sourceSbus = false;
      capDrive = AUTONOMOUS_MAX_DRIVE_RPM;
      capPivot = AUTONOMOUS_MAX_PIVOT_RPM;
      // IMPORTANT: SBUS joystick THROTTLE/STEERING ARE IGNORED in this mode.
      // Motion comes only from ROS /cmd_vel subscriber (cmdVelStatus).
      const bool finite = cmdVelStatus.commandFinite;
      const uint32_t age =
          (cmdVelStatus.receiveCount == 0) ? UINT32_MAX
                                          : (nowMs - cmdVelStatus.lastReceiveMs);
      const bool fresh = cmdVelStatus.commandFresh &&
                         (age <= CMD_VEL_FRESH_TIMEOUT_MS);
      if (finite && fresh) {
        // Differential forward kinematics inverse: (vx, wz) -> wheel speeds.
        const float vx = cmdVelStatus.linearX;
        const float wz = cmdVelStatus.angularZ;
        // Wheel tangent linear speeds (m/s).  REP-103 conventions.
        const float vl = vx - wz * (TRACK_WIDTH_M * 0.5f);
        const float vr = vx + wz * (TRACK_WIDTH_M * 0.5f);
        // Max tangent speed (at capped max drive RPM):
        //   v_max = (maxRpm / 60) * 2 * pi * r
        const float maxRpmFloat = static_cast<float>(AUTONOMOUS_MAX_DRIVE_RPM);
        const float vmax = (maxRpmFloat / 60.0f) * 2.0f * 3.14159265358979323846f * WHEEL_RADIUS_M;
        // Normalize to [-1, +1] for the existing throttle/steering pipeline.
        float leftN  = (vmax > 1e-6f) ? (vl / vmax) : 0.0f;
        float rightN = (vmax > 1e-6f) ? (vr / vmax) : 0.0f;
        if (leftN  >  1.0f) leftN  =  1.0f;
        if (leftN  < -1.0f) leftN  = -1.0f;
        if (rightN >  1.0f) rightN =  1.0f;
        if (rightN < -1.0f) rightN = -1.0f;
        // Convert (leftN, rightN) back to (thrNorm, strNorm) form:
        //   left  = thrNorm + strNorm
        //   right = thrNorm - strNorm
        //   =>  thr = (left+right)/2  ,  str = (left-right)/2
        const float tn = 0.5f * (leftN + rightN);
        const float sn = 0.5f * (leftN - rightN);
        thrNorm = (fabsf(tn) <= MOTION_DEADBAND_NORM) ? 0.0f : tn;
        strNorm = (fabsf(sn) <= MOTION_DEADBAND_NORM) ? 0.0f : sn;
      } else {
        // Cmd vel STALE / missing / non-finite  =>  STOP (neutral = true below).
        thrNorm = 0.0f;
        strNorm = 0.0f;
      }
    } break;
  }
  lastMotionSourceWasSbus = sourceSbus;
  motionLastThrottleNorm = thrNorm;
  motionLastSteeringNorm = strNorm;
  const bool neutral = (thrNorm == 0.0f && strNorm == 0.0f);

  if (!permit || failsafeStopActive() || !sbusRealHealthyFrame(nowMs)) {
    motionArmed = false;
    motionNeutralArmingStartMs = 0;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      requestedMotorRpm[i] = 0;
      motionSlewRequestedRpm[i] = 0;
    }
    return;
  }

  if (!motionArmed) {
    if (neutral) {
      if (motionNeutralArmingStartMs == 0) {
        motionNeutralArmingStartMs = nowMs;
      }
      const uint32_t armElapsed = nowMs - motionNeutralArmingStartMs;
      if (armElapsed >= MOTION_NEUTRAL_ARMING_MS) {
        motionArmed = true;
      }
    } else {
      motionNeutralArmingStartMs = 0;
    }
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      requestedMotorRpm[i] = 0;
      motionSlewRequestedRpm[i] = 0;
    }
    return;
  }

  int16_t tentative[MOTOR_COUNT] = {0, 0, 0, 0};
  if (motionArmed && neutral) {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) tentative[i] = 0;
  } else if (motionArmed) {
    const bool pivot = (thrNorm == 0.0f);
    const int16_t cap = pivot ? capPivot : capDrive;
    float leftN  = thrNorm + strNorm;
    float rightN = thrNorm - strNorm;
    if (leftN  >  1.0f) leftN  =  1.0f;
    if (leftN  < -1.0f) leftN  = -1.0f;
    if (rightN >  1.0f) rightN =  1.0f;
    if (rightN < -1.0f) rightN = -1.0f;
    const int16_t leftRpm  = static_cast<int16_t>(leftN  >= 0.0f ? leftN  * cap + 0.5f : leftN  * cap - 0.5f);
    const int16_t rightRpm = static_cast<int16_t>(rightN >= 0.0f ? rightN * cap + 0.5f : rightN * cap - 0.5f);
    tentative[0] = static_cast<int16_t>(MOTOR_DIRECTION_SIGN[0] * leftRpm);
    tentative[1] = static_cast<int16_t>(MOTOR_DIRECTION_SIGN[1] * leftRpm);
    tentative[2] = static_cast<int16_t>(MOTOR_DIRECTION_SIGN[2] * rightRpm);
    tentative[3] = static_cast<int16_t>(MOTOR_DIRECTION_SIGN[3] * rightRpm);
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      if (tentative[i] >  cap) tentative[i] =  cap;
      if (tentative[i] < -cap) tentative[i] = -cap;
    }
  }

  const uint8_t currIdx = activeMotorIndex;
  bool fullCycleTick = false;
  if (motionLastCycleIndex != currIdx) {
    if (motionLastCycleIndex == MOTOR_COUNT - 1 && currIdx == 0) {
      fullCycleTick = true;
    }
    motionLastCycleIndex = currIdx;
  }

  if (fullCycleTick) {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      const int16_t target = tentative[i];
      int16_t current = motionSlewRequestedRpm[i];
      int16_t delta = target - current;
      if (delta > MOTION_SLEW_LIMIT_PER_CYCLE) delta = MOTION_SLEW_LIMIT_PER_CYCLE;
      if (delta < -MOTION_SLEW_LIMIT_PER_CYCLE) delta = -MOTION_SLEW_LIMIT_PER_CYCLE;
      current += delta;
      motionSlewRequestedRpm[i] = current;
      requestedMotorRpm[i] = current;
    }
  } else {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      requestedMotorRpm[i] = motionSlewRequestedRpm[i];
    }
  }
#else
  (void)nowMs;
#endif
}

static inline bool motorAllStoppedWithin1RpmDecisionOnly() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const int16_t rpm = motorStatus[i].measuredRpm;
    if (rpm < -1 || rpm > 1) return false;
  }
  return true;
}

static inline bool failsafeStopActive() {
  const bool sbusBad = sbusStatus.failsafe || !sbusStatus.connected;
  const bool batBad = batteryStatus.critical;
#if PICO_B_ENABLE_RELAYS
  const uint8_t inhibit = relayNormalInhibit();
#else
  const uint8_t inhibit = RELAY_INHIBIT_DISABLED;
#endif
  const bool inhibitBad = inhibit == RELAY_INHIBIT_SBUS ||
                          inhibit == RELAY_INHIBIT_MOTOR ||
                          inhibit == RELAY_INHIBIT_SYSTEM;
  bool anyCriticalFault = false;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (!motorStatus[i].online || motorStatus[i].errorCode != 0) {
      anyCriticalFault = true;
      break;
    }
  }
  return sbusBad || batBad || inhibitBad || anyCriticalFault;
}

static inline void updateMotorStopState(uint32_t nowMs) {
#if PICO_B_ENABLE_MOTORS
  const bool fs = failsafeStopActive();
  if (fs) {
    if (motorStopState != MotorStopState::FAILSAFE_BRAKE) {
      motorStopState = MotorStopState::FAILSAFE_BRAKE;
      motorStopSettleStartMs = nowMs;
      motorStopStableCycles = 0;
    }
    return;
  }

  if (motorStopState == MotorStopState::FAILSAFE_BRAKE) {
    motorStopState = MotorStopState::ZERO_SETTLE;
    motorStopSettleStartMs = nowMs;
    motorStopStableCycles = 0;
  }

  const bool nonzero = motorHasAnyNonzeroTarget();
  if (nonzero) {
    if (motorStopState != MotorStopState::MOVING) {
      motorStopState = MotorStopState::MOVING;
      motorStopSettleStartMs = nowMs;
      motorStopStableCycles = 0;
    }
    return;
  }

  if (motorStopState == MotorStopState::MOVING) {
    motorStopState = MotorStopState::ZERO_SETTLE;
    motorStopSettleStartMs = nowMs;
    motorStopStableCycles = 0;
    return;
  }

  const uint8_t currIdx = activeMotorIndex;
  bool fullCycleTick = false;
  if (motorStopPrevActiveIndex != currIdx) {
    if (motorStopPrevActiveIndex == MOTOR_COUNT - 1 && currIdx == 0) {
      fullCycleTick = true;
    }
    motorStopPrevActiveIndex = currIdx;
  }

  if (motorStopState == MotorStopState::ZERO_SETTLE) {
    if (fullCycleTick) {
      if (motorAllStoppedWithin1RpmDecisionOnly()) {
        if (motorStopStableCycles < 0xFFU) {
          ++motorStopStableCycles;
        }
      } else {
        motorStopStableCycles = 0;
      }
    }
    const uint32_t elapsed = nowMs - motorStopSettleStartMs;
    const bool stable = motorStopStableCycles >= MOTOR_STOP_STABLE_CYCLES;
    const bool minOk  = elapsed >= MOTOR_STOP_SETTLE_MIN_MS;
    const bool maxHit = elapsed >= MOTOR_STOP_SETTLE_MAX_MS;
    if ((stable && minOk) || maxHit) {
      if (maxHit && !stable) {
        if (motorStopSettleTimeoutCount < 0xFFFFFFFFU) {
          ++motorStopSettleTimeoutCount;
        }
      }
      motorStopState = MotorStopState::BRAKE_HOLD;
    }
  }
#else
  (void)nowMs;
#endif
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
  const int16_t targetRpm = getRuntimeMotorTargetRpm(activeMotorIndex);

  motorSpeedTxFrame[0] = MOTOR_CONFIGS[activeMotorIndex].id;
  motorSpeedTxFrame[1] = 0x64;
  motorSpeedTxFrame[2] = static_cast<uint16_t>(targetRpm) >> 8;
  motorSpeedTxFrame[3] = static_cast<uint16_t>(targetRpm) & 0xFF;
  motorSpeedTxFrame[6] = 0x00;
  motorSpeedTxFrame[7] = getRuntimeMotorBrakeByte();
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
  motorStopState = MotorStopState::ZERO_SETTLE;
  motorStopSettleStartMs = nowMs;
  motorStopStableCycles = 0;
  motorStopPrevActiveIndex = 0xFF;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    requestedMotorRpm[i] = 0;
    motionSlewRequestedRpm[i] = 0;
  }
  motionArmed = false;
  motionNeutralArmingStartMs = 0;
  motionLastThrottleNorm = 0.0f;
  motionLastSteeringNorm = 0.0f;
  motionLastCycleIndex = 0xFF;
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
      ++currentMotorSlotEpoch;
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

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t rc = rcl_init_options_init(&init_options, allocator);
  if (rc == RCL_RET_OK) {
    rc = rcl_init_options_set_domain_id(&init_options, PICO_B_ROS_DOMAIN_ID);
  }
  if (rc == RCL_RET_OK) {
    rc = rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator);
  }
  rcl_init_options_fini(&init_options);
  if (rc != RCL_RET_OK) {
    if (support.context.impl != nullptr) {
      (void)rclc_support_fini(&support);
    }
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

#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
  {
    rcl_subscription_options_t subOpts = rcl_subscription_get_default_options();
    subOpts.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    subOpts.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    subOpts.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    subOpts.qos.depth = 1;
    if (rcl_subscription_init(&ekfDistanceSubscriber,
                              &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                              EKF_DISTANCE_TOPIC,
                              &subOpts) != RCL_RET_OK) {
      return false;
    }
    if (rclc_executor_init(&ekfDistanceExecutor,
                           &support.context,
                           EKF_DISTANCE_EXECUTOR_HANDLE_COUNT,
                           &allocator) != RCL_RET_OK) {
      return false;
    }
    if (rclc_executor_add_subscription(&ekfDistanceExecutor,
                                       &ekfDistanceSubscriber,
                                       &ekfDistanceMsg,
                                       &ekfDistanceCallback,
                                       ON_NEW_DATA) != RCL_RET_OK) {
      return false;
    }
    ekfDistanceSubscriberCreated = true;
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
      diagnosticPage = DiagnosticPage::CMD_VEL;
      return;
    case DiagnosticPage::CMD_VEL:
      diagnosticPage = DiagnosticPage::DRIVE_MODE;
      return;
    case DiagnosticPage::DRIVE_MODE:
      diagnosticPage = DiagnosticPage::EKF_DISTANCE;
      return;
    case DiagnosticPage::EKF_DISTANCE:
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
  setDiagnosticValueUnsigned(5, "budgetDroppedFrames", oledBudgetDroppedFrameCount);
  setDiagnosticValueUnsigned(6, "oledChunkLastUs", oledChunkLastUs);
  setDiagnosticValueUnsigned(7, "oledChunkWindowMaxUs", oledChunkWindowMaxUs);
  setDiagnosticValueUnsigned(8, "oledChunkSkippedCount", oledChunkSkippedCount);
  setDiagnosticValueUnsigned(9, "oledFrameTransferCount", oledFrameTransferCount);
  setDiagnosticValueBool(10, "oledTransferActive",
                         oledTransferState == OledTransferState::TRANSFER_ACTIVE);
  setDiagnosticValueBool(11, "oledRedrawRequested", oledRedrawRequested);
  setDiagnosticValueUnsigned(12, "oledPrepLastUs", oledPrepLastUs);
  setDiagnosticValueUnsigned(13, "oledPrepWindowMaxUs", oledPrepWindowMaxUs);
  setDiagnosticValueUnsigned(14, "oledChunkTxErrorCount", oledChunkTxErrorCount);
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
  {
    const uint8_t brk = getRuntimeMotorBrakeByte();
    const uint8_t st  = static_cast<uint8_t>(motorStopState);
    const uint16_t tout = static_cast<uint16_t>(
        motorStopSettleTimeoutCount > 0xFFFFU ? 0xFFFFU : motorStopSettleTimeoutCount);
    const uint32_t composite4 =
        (static_cast<uint32_t>(brk)  << 24) |
        (static_cast<uint32_t>(st)   << 16) |
        (static_cast<uint32_t>(tout) << 0);
    setDiagnosticValueUnsigned(4, "stBrkTimout", composite4);
  }
  {
    const uint16_t winUs = static_cast<uint16_t>(
        motorTaskWindowMaxUs > 0xFFFFU ? 0xFFFFU : motorTaskWindowMaxUs);
    const uint8_t  cyc   = motorStopStableCycles;
    const uint32_t elRaw = (motorStopSettleStartMs == 0 ||
                            motorStopState == MotorStopState::BRAKE_HOLD ||
                            motorStopState == MotorStopState::FAILSAFE_BRAKE ||
                            motorStopState == MotorStopState::MOVING)
                           ? 0U
                           : static_cast<uint32_t>(nowMs - motorStopSettleStartMs);
    const uint16_t elMs = static_cast<uint16_t>(elRaw > 0x3FFU ? 0x3FFU : elRaw);
    const uint32_t composite5 =
        (static_cast<uint32_t>(winUs) << 16) |
        (static_cast<uint32_t>(cyc)   << 10) |
        (static_cast<uint32_t>(elMs)  << 0);
    setDiagnosticValueUnsigned(5, "winMsStableCy", composite5);
  }
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
  status.level = motionArmed
      ? static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK)
      : static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
  setLiteralString(status.name, CMD_VEL_DIAG_NAME);
  setLiteralString(status.message, motionArmed ? "SBUS_MOTION_ARMED" : "SBUS_MOTION_DISARMED");
  status.values.size = DIAG_VALUE_CAPACITY;

  setDiagnosticValueBool(0, "motionArmed", motionArmed);
  setDiagnosticValueSigned(1, "reqFL_rpm", requestedMotorRpm[0]);
  setDiagnosticValueSigned(2, "reqRL_rpm", requestedMotorRpm[1]);
  setDiagnosticValueSigned(3, "reqFR_rpm", requestedMotorRpm[2]);
  setDiagnosticValueSigned(4, "reqRR_rpm", requestedMotorRpm[3]);

  const int16_t thrI16 = static_cast<int16_t>(motionLastThrottleNorm * 100.0f);
  const int16_t strI16 = static_cast<int16_t>(motionLastSteeringNorm * 100.0f);
  const uint32_t stickPack =
      (static_cast<uint32_t>(static_cast<uint16_t>(thrI16)) << 16) |
      (static_cast<uint32_t>(static_cast<uint16_t>(strI16)) & 0xFFFFU);
  setDiagnosticValueUnsigned(5, "thrStr_x100", stickPack);
#endif
}

void fillDriveModeDiagnostic(uint32_t nowMs,
                             diagnostic_msgs__msg__DiagnosticStatus &status) {
  const DriveMode mode = selectedDriveMode;
  const bool srcSbus = motionSourceIsSbus();
  const bool sbusOk = sbusRealHealthyFrame(nowMs);

  bool cmdFresh = false;
  if (!srcSbus) {
    const bool never = (cmdVelStatus.receiveCount == 0);
    uint32_t ageMs = never ? UINT32_MAX : (nowMs - cmdVelStatus.lastReceiveMs);
    cmdFresh = cmdVelStatus.commandFinite && cmdVelStatus.commandFresh &&
               !never && ageMs <= CMD_VEL_FRESH_TIMEOUT_MS;
  }
  const bool permit = motorMotionPermitted(nowMs) && !failsafeStopActive();

  uint8_t level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  const char *message = "OK";
  if (!sbusOk) {
    level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "SBUS_UNHEALTHY";
  } else if (!srcSbus && !cmdFresh) {
    level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "CMD_STALE";
  } else if (!permit) {
    level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "MOTOR_NOT_PERMITTED";
  }

  status.level = level;
  setLiteralString(status.name, "pico_b/drive_mode");
  setLiteralString(status.message, message);
  status.values.size = DIAG_VALUE_CAPACITY;

  setDiagnosticValue(0, "mode", driveModeText(mode));
  setDiagnosticValueUnsigned(1, "ch6Raw", lastDriveModeCh6Raw);
  setDiagnosticValue(2, "source", srcSbus ? "SBUS" : "ROS");
  setDiagnosticValueBool(3, "sbusHealthy", sbusOk);
  setDiagnosticValueBool(4, "commandFresh", srcSbus ? sbusOk : cmdFresh);
  setDiagnosticValueBool(5, "motorPermit", permit);
}

void fillEkfDistanceDiagnostic(uint32_t nowMs,
                               diagnostic_msgs__msg__DiagnosticStatus &status) {
  bool anyRx = false;
  bool freshFinite = false;
  uint32_t ageMs = 0;
  float distanceM = 0.0f;
  uint32_t rxCount = 0;
  uint32_t rejectCount = 0;
#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
  anyRx = ekfDistanceStatus.distanceReceived;
  if (anyRx) {
    distanceM = ekfDistanceStatus.distanceM;
    freshFinite = ekfDistanceStatus.distanceFinite;
    ageMs = nowMs - ekfDistanceStatus.lastReceiveMs;
    rxCount = ekfDistanceStatus.receiveCount;
    rejectCount = ekfDistanceStatus.rejectCount;
  }
#endif

  uint8_t level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  const char *message = "OK";
#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
  if (!anyRx) {
    level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "WAIT_EKF_DISTANCE";
  } else if (!freshFinite) {
    level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "NON_FINITE_DATA";
  } else if (ageMs > EKF_DISTANCE_STALE_TIMEOUT_MS) {
    level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__WARN);
    message = "STALE";
  }
#else
  level = static_cast<uint8_t>(diagnostic_msgs__msg__DiagnosticStatus__OK);
  message = "DISABLED";
#endif

  status.level = level;
  setLiteralString(status.name, EKF_DISTANCE_DIAG_NAME);
  setLiteralString(status.message, message);
  status.values.size = DIAG_VALUE_CAPACITY;

  setDiagnosticValueFloat(0, "distanceM", distanceM, 2);
  setDiagnosticValueBool(1, "fresh", anyRx && freshFinite && (ageMs <= EKF_DISTANCE_STALE_TIMEOUT_MS));
  setDiagnosticValueUnsigned(2, "ageMs", ageMs);
  setDiagnosticValueUnsigned(3, "rxCount", rxCount);
  setDiagnosticValueUnsigned(4, "rejectCount", rejectCount);
  setDiagnosticValueBool(5, "received", anyRx);
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
  printOledLine(106, 0, driveModeMarker(selectedDriveMode));

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
  printOledLine(106, 0, driveModeMarker(selectedDriveMode));

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
  printOledLine(106, 0, driveModeMarker(selectedDriveMode));

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

void drawDriveModeOledPage(uint32_t nowMs) {
  char line[24];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  printOledLine(0, 0, "DRIVE MODE");
  printOledLine(106, 0, driveModeMarker(selectedDriveMode));

  const DriveMode mode = selectedDriveMode;
  const char *modeLabel = driveModeText(mode);
  strncpy(line, modeLabel, sizeof(line) - 1);
  line[sizeof(line) - 1] = '\0';
  printOledLine(0, 12, line);

  const bool srcSbus = motionSourceIsSbus();
  const char *srcLabel = srcSbus ? "SRC: SBUS" : "SRC: ROS";
  printOledLine(0, 28, srcLabel);

  if (srcSbus) {
    const bool sbusOk = sbusRealHealthyFrame(nowMs);
    const char *sbusLabel = sbusOk ? "SBUS: OK" : "SBUS: FAIL";
    printOledLine(0, 40, sbusLabel);
  } else {
    const bool never = (cmdVelStatus.receiveCount == 0);
    uint32_t ageMs = 0;
    if (!never) {
      ageMs = nowMs - cmdVelStatus.lastReceiveMs;
    }
    const bool fresh = cmdVelStatus.commandFinite &&
                       cmdVelStatus.commandFresh &&
                       !never &&
                       ageMs <= CMD_VEL_FRESH_TIMEOUT_MS;
    const char *cmdLabel = fresh ? "CMD: FRESH" : "CMD: STALE";
    printOledLine(0, 40, cmdLabel);
    if (!fresh) {
      printOledLine(0, 52, "MOTOR: STOP");
    } else {
      const bool permit = motorMotionPermitted(nowMs) && !failsafeStopActive();
      printOledLine(0, 52, permit ? "MOTOR: PERMIT" : "MOTOR: STOP");
    }
  }
}
#endif

#if PICO_B_ENABLE_OLED
void drawTripOledPage(uint32_t nowMs) {
  char line[24];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  printOledLine(0, 0, "TRIP");
  printOledLine(106, 0, driveModeMarker(selectedDriveMode));

  printOledLine(0, 14, "DIST:");

  float distVal = 0.0f;
  bool anyRx = false;
  bool freshFinite = false;
  uint32_t ageMs = 0;
#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
  anyRx = ekfDistanceStatus.distanceReceived;
  if (anyRx) {
    distVal = ekfDistanceStatus.distanceM;
    freshFinite = ekfDistanceStatus.distanceFinite;
    ageMs = nowMs - ekfDistanceStatus.lastReceiveMs;
  }
#endif

  if (!anyRx) {
    printOledLine(0, 26, "WAIT...");
  } else if (!freshFinite) {
    printOledLine(0, 26, "ERR DATA");
  } else {
    const float absVal = fabsf(distVal);
    const char sign = (distVal >= 0.0f) ? '+' : '-';
    int written;
    if (absVal < 100.0f) {
      written = snprintf(line, sizeof(line), "%c%.2f m", sign, absVal);
    } else if (absVal < 1000.0f) {
      written = snprintf(line, sizeof(line), "%c%.1f m", sign, absVal);
    } else {
      written = snprintf(line, sizeof(line), "%c%.0f m", sign, absVal);
    }
    if (written < 0 || static_cast<size_t>(written) >= sizeof(line)) {
      line[sizeof(line) - 1] = '\0';
    }
    display.setTextSize(2);
    printOledLine(4, 26, line);
    display.setTextSize(1);
  }

  printOledLine(0, 54, "EKF:");
#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
  if (!anyRx) {
    printOledLine(36, 54, "WAIT");
  } else {
    const bool stale = (ageMs > EKF_DISTANCE_STALE_TIMEOUT_MS);
    printOledLine(36, 54, stale ? "STALE" : "OK");
  }
#else
  printOledLine(36, 54, "DISABLED");
#endif
}
#endif

#if PICO_B_ENABLE_OLED
static inline void prepareOledFramebufferForCurrentPage(uint32_t nowMs) {
  display.clearDisplay();
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
    case OledPage::DRIVE_MODE:
      drawDriveModeOledPage(nowMs);
      break;
    case OledPage::EKF_TRIP:
      drawTripOledPage(nowMs);
      break;
    default:
      drawSystemOledPage(nowMs);
      break;
  }
}
#endif

#if PICO_B_ENABLE_OLED && PICO_B_ENABLE_MOTORS
static inline void serviceOledSlotWork() {
  if (motorTaskState != MotorTaskState::IDLE) {
    return;
  }
  const uint32_t nowUs = micros();
  const uint32_t deadlineUs = motorDueUs[activeMotorIndex];
  const int32_t remaining =
      static_cast<int32_t>(deadlineUs - nowUs);
  if (remaining < 0) {
    return;
  }
  if (oledLastServicedSlotEpoch == currentMotorSlotEpoch) {
    return;
  }

  if (oledFramePrepPending) {
    if (remaining < static_cast<int32_t>(OLED_PREP_SAFE_BUDGET_US)) {
      return;
    }
    const uint32_t t0 = micros();
    const uint32_t nowMs = millis();
    prepareOledFramebufferForCurrentPage(nowMs);
    const uint32_t dt = micros() - t0;
    oledPrepLastUs = dt;
    if (oledPrepLastUs > oledPrepWindowMaxUs) {
      oledPrepWindowMaxUs = oledPrepLastUs;
    }
    oledFramePrepPending = false;
    oledChunkIndex = 0;
    oledFrameCumulativeUs = 0;
    oledTransferState = OledTransferState::TRANSFER_ACTIVE;
    oledLastServicedSlotEpoch = currentMotorSlotEpoch;
    return;
  }

  if (oledTransferState == OledTransferState::TRANSFER_ACTIVE) {
    if (remaining < static_cast<int32_t>(OLED_CHUNK_SAFE_BUDGET_US)) {
      ++oledChunkSkippedCount;
      return;
    }
    const uint32_t chunkStartUs = micros();
    uint8_t rc = 0;

    if (oledChunkIndex == 0) {
      static const uint8_t PROGMEM setupCmds1[] = {
          0x21, 0, 127,
          0x22, 0, 7,
      };
      Wire.beginTransmission(OLED_ADDRESS);
      Wire.write(static_cast<uint8_t>(0x00));
      for (uint8_t i = 0; i < sizeof(setupCmds1); ++i) {
        Wire.write(pgm_read_byte(&setupCmds1[i]));
      }
      rc = Wire.endTransmission();
    } else {
      const uint8_t *fb = display.getBuffer();
      const uint16_t dataIdx =
          static_cast<uint16_t>(oledChunkIndex - OLED_SETUP_CHUNK_COUNT) *
          OLED_CHUNK_BYTES;
      uint16_t thisChunk = OLED_CHUNK_BYTES;
      if (dataIdx + thisChunk > OLED_TOTAL_FRAMEBUFFER_BYTES) {
        thisChunk = OLED_TOTAL_FRAMEBUFFER_BYTES - dataIdx;
      }
      Wire.beginTransmission(OLED_ADDRESS);
      Wire.write(static_cast<uint8_t>(0x40));
      for (uint16_t i = 0; i < thisChunk; ++i) {
        Wire.write(fb[dataIdx + i]);
      }
      rc = Wire.endTransmission();
    }

    const uint32_t dt = micros() - chunkStartUs;
    oledChunkLastUs = dt;
    if (oledChunkLastUs > oledChunkWindowMaxUs) {
      oledChunkWindowMaxUs = oledChunkLastUs;
    }
    oledFrameCumulativeUs += dt;
    oledLastServicedSlotEpoch = currentMotorSlotEpoch;

    if (rc != 0) {
      ++oledChunkTxErrorCount;
      return;
    }

    ++oledChunkIndex;
    if (oledChunkIndex >= OLED_TOTAL_CHUNKS_PER_FRAME) {
      oledTransferState = OledTransferState::IDLE;
      oledLastUpdateUs = oledFrameCumulativeUs;
      if (oledLastUpdateUs > oledWindowMaxUs) {
        oledWindowMaxUs = oledLastUpdateUs;
      }
      ++oledFrameTransferCount;
      const uint32_t nowMs = millis();
      updateOledRate(nowMs);
      lastOledUpdateMs = nowMs;
      if (oledRedrawRequested) {
        oledRedrawRequested = false;
        oledFramePrepPending = true;
      }
    }
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

  const bool periodDue =
      periodElapsed(nowMs, lastOledUpdateMs, OLED_UPDATE_PERIOD_MS);
  if (periodDue) {
    oledRedrawRequested = true;
  }

  if (oledRedrawRequested &&
      oledTransferState == OledTransferState::IDLE &&
      !oledFramePrepPending) {
    oledRedrawRequested = false;
    oledFramePrepPending = true;
  }

#if PICO_B_ENABLE_MOTORS
  serviceOledSlotWork();
#endif

#else
  (void)nowMs;
#endif
}

void setupOled() {
#if PICO_B_ENABLE_OLED
  if (oledSplashShown) {
    return;
  }
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
  oledSplashShown = true;
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
    case DiagnosticPage::DRIVE_MODE:
      fillDriveModeDiagnostic(nowMs, status);
      break;
    case DiagnosticPage::EKF_DISTANCE:
      fillEkfDistanceDiagnostic(nowMs, status);
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
#if PICO_B_ENABLE_EKF_DISTANCE_SUBSCRIBER
      {
        const rcl_ret_t execRet = rclc_executor_spin_some(&ekfDistanceExecutor, 0);
        (void)execRet;
      }
#endif
      if (rosSessionError) {
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

  const uint32_t nowMs = millis();
  lastBatteryMeasurementMs = nowMs;
  initializeRelayControl(nowMs);
  initializeCameraServo(nowMs);
  initializeMotorTask(nowMs);
  measureBattery(nowMs);
}

void loop() {
  uint32_t nowMs = millis();
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
  updateMotionCommandAndArming(nowMs);
  motorTask(nowMs);
  updateMotorStopState(nowMs);
#if PICO_B_ENABLE_OLED && PICO_B_ENABLE_MOTORS
  serviceOledSlotWork();
#endif
  if (periodElapsed(nowMs, lastBatteryMeasurementMs,
                    BATTERY_MEASUREMENT_PERIOD_MS)) {
    measureBattery(nowMs);
  }
  nowMs = millis();
  motorTask(nowMs);
#if PICO_B_ENABLE_OLED && PICO_B_ENABLE_MOTORS
  serviceOledSlotWork();
#endif
  relayTask(nowMs);
  if (motorAllowsSlowTasks()) {
    cameraServoTask(nowMs);
  }
  updateOled(nowMs);
  rosTask(millis());
}
