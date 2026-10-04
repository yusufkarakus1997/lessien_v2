# PICO_A_MICROROS_SENSOR_NODE

Independent Pico A micro-ROS Jazzy sensor node for Raspberry Pi Pico 2 /
RP2350 and Arduino-Pico 5.6.0.

## Status

- Jazzy heartbeat hardware gate: PASS
- Sensor node compile/link gate: PASS for all requested feature combinations
- Final G3 upload: PASS
- Final G3 combined hardware validation: 60 minutes PASS

## Protected Golden Inputs

These files were hash-checked and were not edited:

```text
560e7861c881295e794c785c300f7e1dc7ca5f5186e54a29b9be4838225593a1  PICO_A_IMU_TASK_TEST/PICO_A_IMU_TASK_TEST.ino
d66a0800aefd64d5e9ba88e4df81872de5db469202d0a2e1d5315585474ff999  PICO_A_ULTRASONIC_5_FILTER_TEST/PICO_A_ULTRASONIC_5_FILTER_TEST.ino
3b114997b49133671a8a498b690ebb6569db896a1c4f0b3b32a3a26db8bd08dc  12_task.ino
982833adc2696128e60987b867a835dc31c217416c6b8b38d1275ab9c05ab389  PICO_SENSOR_SPI_CHUNK_INTEGRATION/PICO_A_SENSOR_CHUNK_SLAVE/PICO_A_SENSOR_CHUNK_SLAVE.ino
b69890df91deac8df3f40ef98e862322ae0b029bdaa79c579a291ea18006f1b6  PICO_SENSOR_SPI_CHUNK_INTEGRATION/PICO_B_SENSOR_CHUNK_MASTER/PICO_B_SENSOR_CHUNK_MASTER.ino
```

## micro-ROS Library

- Library path:
  `/home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy`
- Jazzy tag: `v2.0.8-jazzy`
- Source commit: `f5fe0f6ab75c2b5ac2de828a2471e62a1b17b009`
- Arduino-Pico core: `rp2040:rp2040` 5.6.0
- Toolchain: `/home/yusuf/.arduino15/packages/rp2040/tools/pqt-gcc/4.1.0-1aec55e/bin/arm-none-eabi-*`
- ABI flags: `-mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp -mfloat-abi=softfp -mcmse`
- Transport: `RMW_UXRCE_TRANSPORT=custom`
- Publisher pool: `RMW_UXRCE_MAX_PUBLISHERS=10`
- Current `libmicroros.a` SHA256:
  `34930364ef68e906575300d34b8aa067a280fb4e6c12d845d39dbc582abd1277`

The earlier heartbeat-only archive hash was
`38d60dc2599a1fd138a6bbe40ee6ebf357e0dab6d070600fc6ad3e98ed220b04`.
It was rebuilt from the same Jazzy snapshot because the sensor node needs more
than two publishers.

## Feature Flags

Defaults in `sensor_node_app.cpp` are:

```cpp
constexpr bool ENABLE_IMU = true;
constexpr bool ENABLE_ULTRASONICS = false;
constexpr bool ENABLE_GYRO_REPORT = true;
constexpr bool ENABLE_ROS_IMU_PUBLISHER = true;
constexpr bool ENABLE_ROS_RANGE_PUBLISHERS = false;
constexpr bool ENABLE_ROS_DIAGNOSTICS = true;
```

Build-time override macros:

- `-DPICO_A_ENABLE_IMU=1`
- `-DPICO_A_ENABLE_GYRO_REPORT=1`
- `-DPICO_A_ENABLE_ULTRASONICS=0`
- `-DPICO_A_ENABLE_ROS_IMU_PUBLISHER=1`
- `-DPICO_A_ENABLE_ROS_RANGE_PUBLISHERS=0`
- `-DPICO_A_ENABLE_ROS_DIAGNOSTICS=1`

## Topics

- `/pico_a/heartbeat` (`std_msgs/msg/UInt32`, best effort, 2 Hz)
- `/imu/data` (`sensor_msgs/msg/Imu`, best effort)
- `/pico_a/range/front` (`sensor_msgs/msg/Range`, best effort)
- `/pico_a/range/top` (`sensor_msgs/msg/Range`, best effort)
- `/pico_a/range/left` (`sensor_msgs/msg/Range`, best effort)
- `/pico_a/range/right` (`sensor_msgs/msg/Range`, best effort)
- `/pico_a/range/rear` (`sensor_msgs/msg/Range`, best effort)
- `/pico_a/obstacle_stop` (`std_msgs/msg/Bool`, best effort)
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`, best effort)

## Compile Commands

IMU-only:

```bash
/tmp/arduino-cli-bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property "compiler.cpp.extra_flags=-DPICO_A_ENABLE_IMU=1 -DPICO_A_ENABLE_ULTRASONICS=0" \
  --build-path /tmp/pico_a_sensor_node_imu_only \
  /home/yusuf/12_task/PICO_A_MICROROS_SENSOR_NODE
```

Ultrasonic-only:

```bash
/tmp/arduino-cli-bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property "compiler.cpp.extra_flags=-DPICO_A_ENABLE_IMU=0 -DPICO_A_ENABLE_ULTRASONICS=1" \
  --build-path /tmp/pico_a_sensor_node_ultra_only \
  /home/yusuf/12_task/PICO_A_MICROROS_SENSOR_NODE
```

Combined:

```bash
/tmp/arduino-cli-bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property "compiler.cpp.extra_flags=-DPICO_A_ENABLE_IMU=1 -DPICO_A_ENABLE_ULTRASONICS=1" \
  --build-path /tmp/pico_a_sensor_node_combined \
  /home/yusuf/12_task/PICO_A_MICROROS_SENSOR_NODE
```

All builds selected:

```text
micro_ros_arduino 2.0.8-jazzy /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy
Using precompiled library in .../src/cortex-m33/softfp
```

## Compile Results

| Combination | Flash | RAM |
| --- | ---: | ---: |
| IMU=true, ULTRASONICS=false | 133808 bytes (3%) | 44820 bytes (8%) |
| IMU=false, ULTRASONICS=true | 127628 bytes (3%) | 40688 bytes (7%) |
| IMU=true, ULTRASONICS=true | 139816 bytes (3%) | 49668 bytes (9%) |

Final G3 artifact:

```text
release_artifacts/pico_a_g3/PICO_A_MICROROS_SENSOR_NODE_G3_JAZZY_RP2350.uf2
SHA256: 5a24f88edc529ce2a294bb789b79a722df179b138beb8f93a9c02b9f2d58f0a3
```

## Final G3 Hardware Validation

Final combined hardware run passed for 60 minutes:

- `/pico_a/heartbeat`: `2 Hz`
- `/imu/data`: approximately `50 Hz`
- orientation raw: `50 Hz`
- calibrated gyro raw: `49-50 Hz`
- five Range topics: approximately `11.1 Hz` each
- `/pico_a/obstacle_stop`: `10 Hz`
- `/diagnostics`: `2 Hz`
- `publishErrors=0`
- `reconnects=0`
- `timeSynced=true`
- all five ultrasonic sensors `VALID`
- `obstacle_stop` became `true` for a near obstacle and returned to `false`
  when the obstacle was moved away
- U1 had 63 transient timeouts in 60 minutes, about 0.16 percent, recovered
  automatically, and remained `VALID`
- USB disconnect/re-enumeration: `0`

`sensor_msgs/msg/Imu.linear_acceleration` is not provided by this firmware.
`linear_acceleration_covariance[0] = -1.0` marks linear acceleration as absent.

## Reset Audit Update

Hardware symptom under the first sensor-node firmware was physical USB
disconnect/re-enumeration, not only an Agent reconnect. The stable heartbeat
firmware and stable golden IMU test narrowed this to sensor-node integration
behavior.

Static audit found no `watchdog_enable`, `watchdog_reboot`, `rp2040.reboot`,
`reset_usb_boot`, `panic`, `assert`, `abort`, `error_loop`, `RCCHECK`,
`RCSOFTCHECK`, `TinyUSB`, or `USBDevice` calls in this sketch.

The reset-matching fault path was the RUNNING-state Agent ping. In micro-ROS
Arduino the default weak transport close calls `Serial.end()`. Repeated
`rmw_uros_ping_agent()` while RUNNING can open/close the custom transport and
therefore drop USB CDC, matching the observed repeated Pico 2 USB
disconnect/re-enumeration. The sketch now provides strong
`arduino_transport_open/read/write/close` symbols locally. `open` starts
Serial once, and `close` intentionally does not call `Serial.end()`. Agent ping
is only used in `WAIT_AGENT`, not in `RUNNING`.

The old hardware binary did not contain a persistent boot-stage marker, so its
exact reset stage cannot be recovered after the fact. The current firmware adds
a `.noinit` boot marker, `watchdog_caused_reboot()` capture, and LED blink of
the previous boot stage without USB Serial debug text:

| Stage | Meaning |
| ---: | --- |
| 1 | transport configured |
| 2 | sensor shared state ready |
| 3 | core1 released |
| 4 | allocator/support ready |
| 5 | node ready |
| 6 | publishers ready |
| 7 | time sync ready |
| 8 | running |

Core1 is now held until core0 has initialized shared state and released it with
an atomic flag. Diagnostics storage is global/static; `DiagnosticArray`,
`DiagnosticStatus`, `KeyValue`, and string buffers are assigned once from fixed
storage and are not large local stack objects.

RCL error paths are non-rebooting:

| Function/path | Failure result | State transition | Reboot behavior |
| --- | --- | --- | --- |
| `rmw_uros_ping_agent` in `WAIT_AGENT` | not `RMW_RET_OK` | stay `WAIT_AGENT` | none |
| `rclc_support_init` | not `RCL_RET_OK`, step 1 | destroy partial entities, `WAIT_AGENT` | none |
| `rclc_node_init_default` | not `RCL_RET_OK`, step 2 | destroy partial entities, `WAIT_AGENT` | none |
| heartbeat publisher init | not `RCL_RET_OK`, step 3 | destroy partial entities, `WAIT_AGENT` | none |
| IMU publisher init | not `RCL_RET_OK`, step 4 | destroy partial entities, `WAIT_AGENT` | none |
| range publisher init | not `RCL_RET_OK`, step 10..14 | destroy partial entities, `WAIT_AGENT` | none |
| obstacle publisher init | not `RCL_RET_OK`, step 20 | destroy partial entities, `WAIT_AGENT` | none |
| diagnostics publisher init | not `RCL_RET_OK`, step 30 | destroy partial entities, `WAIT_AGENT` | none |
| `rmw_uros_sync_session` or epoch sync | failed sync | `DESTROY_ENTITIES`, then `WAIT_AGENT` | none |
| `rcl_publish` | not `RCL_RET_OK` | mark session error, `DESTROY_ENTITIES`, then `WAIT_AGENT` | none |

Isolation build matrix, all with Pico 2 `--warnings all` and the Jazzy library
path selected explicitly:

| Build | IMU | Ultrasonics | ROS IMU | ROS Range | ROS Diagnostics | Result | Flash | RAM |
| --- | --- | --- | --- | --- | --- | --- | ---: | ---: |
| A | false | false | false | false | false | PASS | 115216 bytes (2%) | 31424 bytes (5%) |
| B | false | false | false | false | true | PASS | 121908 bytes (2%) | 36240 bytes (6%) |
| C | true | false | false | false | false | PASS | 121544 bytes (2%) | 35860 bytes (6%) |
| D | true | false | true | false | false | PASS | 129236 bytes (3%) | 42316 bytes (8%) |
| E | true | false | true | false | true | PASS | 134272 bytes (3%) | 45380 bytes (8%) |

The build output selects:

```text
micro_ros_arduino 2.0.8-jazzy /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy
Using precompiled library in /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy/src/cortex-m33/softfp
```

Vendor warnings from the SparkFun BNO08x C sources may still appear under
`--warnings all`. No warning was introduced from the sensor-node project source.

First hardware upload target is build A only:

```bash
/tmp/arduino-cli-bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property "compiler.cpp.extra_flags=-DPICO_A_ENABLE_IMU=0 -DPICO_A_ENABLE_ULTRASONICS=0 -DPICO_A_ENABLE_ROS_IMU_PUBLISHER=0 -DPICO_A_ENABLE_ROS_RANGE_PUBLISHERS=0 -DPICO_A_ENABLE_ROS_DIAGNOSTICS=0" \
  --build-path /tmp/pico_a_sensor_node_A \
  /home/yusuf/12_task/PICO_A_MICROROS_SENSOR_NODE
```

## Final Candidate Update

Final candidate adds calibrated gyro to the existing orientation-only path
without changing the USB transport fix or the golden ultrasonic state machine.

BNO08x reports:

- Game Rotation Vector: `enableGameRotationVector(20)`, 50 Hz target
- Calibrated Gyroscope: `enableGyro(20)`, 50 Hz target
- Linear acceleration is not enabled
- 100 Hz reports are not enabled

SparkFun BNO08x 1.0.6 source check:

- `enableGyro()` enables `SENSOR_REPORTID_GYROSCOPE_CALIBRATED`
- event parser uses `SH2_GYROSCOPE_CALIBRATED`
- `sh2_SensorValue.h` documents gyroscope units as `rad/s`
- `/imu/data.angular_velocity` therefore uses gyro x/y/z directly without
  degree-to-radian conversion

`/imu/data` now publishes only when orientation and gyro are both valid, finite,
fresh at `<=40 ms`, paired within `<=25 ms`, both sequences advanced, and ROS
time sync is valid. Header timestamp remains the real orientation sample
timestamp.

Angular velocity covariance uses provisional diagonal values:

```text
angular_velocity_covariance[0,4,8] = 0.0004
linear_acceleration_covariance[0] = -1.0
```

Publisher count for G3 remains below the Jazzy library pool:

```text
heartbeat 1 + IMU 1 + Range 5 + obstacle 1 + diagnostics 1 = 9
RMW_UXRCE_MAX_PUBLISHERS=10
```

G1 compile:

```bash
/tmp/arduino-cli-bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property "compiler.cpp.extra_flags=-DPICO_A_ENABLE_IMU=1 -DPICO_A_ENABLE_GYRO_REPORT=1 -DPICO_A_ENABLE_ULTRASONICS=0 -DPICO_A_ENABLE_ROS_IMU_PUBLISHER=1 -DPICO_A_ENABLE_ROS_RANGE_PUBLISHERS=0 -DPICO_A_ENABLE_ROS_DIAGNOSTICS=0" \
  --build-path /tmp/pico_a_sensor_node_G1 \
  /home/yusuf/12_task/PICO_A_MICROROS_SENSOR_NODE
```

G2 compile:

```bash
/tmp/arduino-cli-bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property "compiler.cpp.extra_flags=-DPICO_A_ENABLE_IMU=0 -DPICO_A_ENABLE_GYRO_REPORT=0 -DPICO_A_ENABLE_ULTRASONICS=1 -DPICO_A_ENABLE_ROS_IMU_PUBLISHER=0 -DPICO_A_ENABLE_ROS_RANGE_PUBLISHERS=1 -DPICO_A_ENABLE_ROS_DIAGNOSTICS=0" \
  --build-path /tmp/pico_a_sensor_node_G2 \
  /home/yusuf/12_task/PICO_A_MICROROS_SENSOR_NODE
```

G3 final compile:

```bash
/tmp/arduino-cli-bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property "compiler.cpp.extra_flags=-DPICO_A_ENABLE_IMU=1 -DPICO_A_ENABLE_GYRO_REPORT=1 -DPICO_A_ENABLE_ULTRASONICS=1 -DPICO_A_ENABLE_ROS_IMU_PUBLISHER=1 -DPICO_A_ENABLE_ROS_RANGE_PUBLISHERS=1 -DPICO_A_ENABLE_ROS_DIAGNOSTICS=1" \
  --build-path /tmp/pico_a_sensor_node_G3 \
  /home/yusuf/12_task/PICO_A_MICROROS_SENSOR_NODE
```

G build results:

| Build | Flash | RAM |
| --- | ---: | ---: |
| G1 | 130252 bytes (3%) | 45168 bytes (8%) |
| G2 | 123064 bytes (2%) | 40864 bytes (7%) |
| G3 | 141112 bytes (3%) | 53084 bytes (10%) |

All G builds selected the Jazzy library path:

```text
micro_ros_arduino 2.0.8-jazzy /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy
Using precompiled library in /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy/src/cortex-m33/softfp
```

G3 ELF contains the local strong transport symbols:

```text
arduino_transport_open
arduino_transport_close
arduino_transport_read
arduino_transport_write
```

`arduino_transport_close` disassembles to `return true`; it does not call
`Serial.end()`.

## G1 Timing Diagnostic Update

G1 hardware reported stable USB/micro-ROS transport but `/imu/data` was about
31.9 Hz instead of the 50 Hz target. G2/G3 testing is paused until G1 timing is
verified.

The G1 timing build keeps:

- Game Rotation Vector target: `enableGameRotationVector(20)`
- Calibrated Gyro target: `enableGyro(20)`
- I2C clock: `100 kHz`
- USB strong transport override
- no Agent ping in `RUNNING`
- ultrasonics disabled

SparkFun BNO08x 1.0.6 source confirms both report APIs multiply the argument by
1000 before `enableReport`, so `20` requests a 20 ms report period.

The BNO service path now drains the event queue per `imuTask()` call with hard
bounds:

```text
max events per service = 8
max drain budget = 1000 us
```

Each drained event is parsed by its own report ID. ROS publish is still outside
the BNO service path and outside any ISR.

`/imu/data` publish is now orientation-centered:

- a new orientation sequence is required
- the same orientation sequence is never published twice
- the latest valid gyro sample is used
- gyro sequence no longer has to advance for every orientation publish
- orientation age and gyro age must each be `<=40 ms`
- orientation/gyro skew must be `<=25 ms`
- ROS time sync must be valid

G1 `/diagnostics` includes the lightweight 2 Hz fields needed to split raw BNO
rate from publish gating:

```text
imuOrientationRawHz
imuGyroRawHz
imuPublishHz
orientationSequence
gyroSequence
orientationAgeUs
gyroAgeUs
orientationGyroSkewUs
bnoEventsPerService
bnoEventDrainMax
rejectOrientationNotNew
rejectGyroNotNew
rejectFreshness
rejectSkew
rejectInvalid
rejectRosTime
publishError
sensorTaskMaxUs
loopMaxUs
```

G1 diagnostic compile:

```bash
/tmp/arduino-cli-bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property "compiler.cpp.extra_flags=-DPICO_A_ENABLE_IMU=1 -DPICO_A_ENABLE_GYRO_REPORT=1 -DPICO_A_ENABLE_ULTRASONICS=0 -DPICO_A_ENABLE_ROS_IMU_PUBLISHER=1 -DPICO_A_ENABLE_ROS_RANGE_PUBLISHERS=0 -DPICO_A_ENABLE_ROS_DIAGNOSTICS=1" \
  --build-path /tmp/pico_a_sensor_node_G1_diag \
  /home/yusuf/12_task/PICO_A_MICROROS_SENSOR_NODE
```

G1 diagnostic build result:

```text
Sketch uses 136024 bytes (3%) of program storage space.
Global variables use 44376 bytes (8%) of dynamic memory.
micro_ros_arduino 2.0.8-jazzy /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy
```

## Bounded Diagnostics Update

G1 hardware Agent logs showed that the previous large diagnostics message could
trigger a best-effort publish failure and session teardown. Diagnostics are now
paged and noncritical.

Every `/diagnostics` publish is bounded to:

```text
DiagnosticArray.status.size = 1
DiagnosticStatus.values.size = 6
unused DiagnosticStatus values.size = 0
static storage only
```

G1 pages rotate every 500 ms:

```text
page=imu_rates
orientationRawHz, gyroRawHz, imuPublishHz, orientationSequence, gyroSequence

page=imu_timing
orientationAgeUs, gyroAgeUs, orientationGyroSkewUs, bnoEventsPerService, bnoEventDrainMax

page=imu_reject_a
rejectOrientationNotNew, rejectGyroNotNew, rejectFreshness, rejectSkew, rejectInvalid

page=imu_reject_b
rejectRosTime, publishError, sensorTaskMaxUs, loopMaxUs, reconnectCount
```

When ultrasonics are enabled later, ultrasonic and micro-ROS diagnostics use the
same one-status/six-KeyValue paging limit.

Diagnostics publish failures increment `diagnosticPublishError` only. They do
not set `rosSessionError` and do not destroy heartbeat or IMU entities.

Current G1 diagnostic build:

```text
Sketch uses 136032 bytes (3%) of program storage space.
Global variables use 44376 bytes (8%) of dynamic memory.
```

## Verification

Archive symbols verified with `arm-none-eabi-nm`:

```text
rcutils_get_default_allocator
rclc_support_init
rclc_node_init_default
rclc_publisher_init_best_effort
rcl_publish
rmw_uros_set_custom_transport
sensor_msgs/Imu C typesupport
sensor_msgs/Range C typesupport
diagnostic_msgs/DiagnosticArray C typesupport
std_msgs/Bool C typesupport
std_msgs/UInt32 C typesupport
```

Final IMU-only ELF also contains micro-ROS publish/time/typesupport symbols.

`cppcheck --language=c++ --std=c++17` completed for the project sources with no
project warning/error after suppressing external Arduino/vendor include noise.

## Runtime Notes

- Serial output is not used by the sketch; USB Serial is reserved for
  micro-ROS custom transport.
- IMU code preserves the golden BNO08x pins, 100 kHz I2C clock, 8 ms polling,
  game rotation vector report, quaternion validation/filtering, stale/backoff
  recovery, and core1 acquisition.
- `sensor_msgs/Imu` publishes orientation and calibrated angular velocity.
  Linear acceleration is not provided; `linear_acceleration_covariance[0]` is
  `-1.0`.
- Ultrasonic code preserves the five sensor pins, 2 us low pulse, 10 us high
  pulse, 15 ms echo timeout, 3 ms quiet gap, 18 ms slot, 90 ms scan period,
  median/EMA filtering, hold/unknown recovery, and ISR edge capture.
- Combined hardware testing should wait until IMU-only and ultrasonic-only
  hardware tests both pass.
