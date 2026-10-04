# Pico B micro-ROS Control Node B4

Scope: B4 keeps the B3 heartbeat, reconnect, SBUS telemetry, battery voltage
telemetry, bounded diagnostics, and SSD1306 OLED behavior, and adds DDSM115
zero-RPM motor communication plus motor telemetry. Optional B4 feature flags add
two active-LOW relay outputs and an RDS3235 FPV camera servo. GP12 and GP13
stay latched HIGH/OFF from boot. SPI is not initialized. SBUS, battery, ROS,
OLED, relay, and servo logic never change the compile-time motor target.

OLED defaults to enabled with `PICO_B_ENABLE_OLED=1`. Build with
`PICO_B_ENABLE_OLED=0` for the B2A regression image without Wire/SSD1306 init or
OLED updates.

Motor communication defaults to enabled with `PICO_B_ENABLE_MOTORS=1` and
`PICO_B_ENABLE_ROS_MOTOR_PUBLISHER=1`. B3A always sends 0 RPM with active brake
byte `0xFF`; there is no `/cmd_vel`, subscriber, service, client, ID setter, or
broadcast command in this firmware.
`PICO_B_MOTOR_CYCLE_HZ` selects the compile-time motor cycle rate and accepts
only `10`, `15`, or `20`. OLED is intentionally disabled for the first 15/20 Hz
rate tests so SSD1306 framebuffer transfers do not affect the motor scheduler.

Relay and camera-servo support are compile-time optional and default safe/off:

- `PICO_B_ENABLE_RELAYS=0` by default. When enabled, relay 1 is GP12 and relay 2
  is GP13, both active LOW (`HIGH=OFF`, `LOW=ON`).
- `PICO_B_ENABLE_CAMERA_SERVO=0` by default. When enabled, GP6 drives the
  RDS3235 FPV camera servo after preparing the center pulse first.
- B3 regression builds must use `PICO_B_ENABLE_RELAYS=0` and
  `PICO_B_ENABLE_CAMERA_SERVO=0`.
- B4 final builds use `PICO_B_ENABLE_RELAYS=1` and
  `PICO_B_ENABLE_CAMERA_SERVO=1`.

RDS3235 calibration is fixed and must not be changed:

- logical `0 deg -> 550 us`
- logical `90 deg -> 1570 us`
- logical `180 deg -> 2450 us`
- attach limit `500-2500 us`
- mapping is two-piece because the center is asymmetric.

## Build

B2B regression build with motors disabled:

```bash
/home/yusuf/.local/bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --clean \
  --verbose \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property compiler.cpp.extra_flags="-DPICO_B_ENABLE_MOTORS=0 -DPICO_B_ENABLE_ROS_MOTOR_PUBLISHER=0 -DPICO_B_ENABLE_OLED=1" \
  --build-path /tmp/pico_b_microros_B2B_regression_build \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE
```

B3A motor-zero build:

```bash
/home/yusuf/.local/bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --clean \
  --verbose \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property compiler.cpp.extra_flags="-DPICO_B_ENABLE_MOTORS=1 -DPICO_B_ENABLE_ROS_MOTOR_PUBLISHER=1 -DPICO_B_ENABLE_OLED=1" \
  --build-path /tmp/pico_b_microros_B3A_motor_zero_build \
	  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE
```

B3A motor rate experiment builds:

```bash
/home/yusuf/.local/bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --clean \
  --verbose \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property compiler.cpp.extra_flags="-DPICO_B_MOTOR_CYCLE_HZ=10 -DPICO_B_ENABLE_MOTORS=1 -DPICO_B_ENABLE_ROS_MOTOR_PUBLISHER=1 -DPICO_B_ENABLE_OLED=0" \
  --build-path /tmp/pico_b_B3A_motor_10hz_oled_off \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE

/home/yusuf/.local/bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --clean \
  --verbose \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property compiler.cpp.extra_flags="-DPICO_B_MOTOR_CYCLE_HZ=15 -DPICO_B_ENABLE_MOTORS=1 -DPICO_B_ENABLE_ROS_MOTOR_PUBLISHER=1 -DPICO_B_ENABLE_OLED=0" \
  --build-path /tmp/pico_b_B3A_motor_15hz_oled_off \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE

/home/yusuf/.local/bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --clean \
  --verbose \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property compiler.cpp.extra_flags="-DPICO_B_MOTOR_CYCLE_HZ=20 -DPICO_B_ENABLE_MOTORS=1 -DPICO_B_ENABLE_ROS_MOTOR_PUBLISHER=1 -DPICO_B_ENABLE_OLED=0" \
  --build-path /tmp/pico_b_B3A_motor_20hz_oled_off \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE
```

B3 regression after B4 integration, with relay and camera servo disabled:

```bash
/home/yusuf/.local/bin/arduino-cli compile --clean --warnings all \
  --fqbn rp2040:rp2040:rpipico2 \
  --libraries /home/yusuf/12_task/microros_rp2350_jazzy_port \
  --build-path /tmp/pico_b_B3_regression_no_relays_no_servo \
  --build-property compiler.cpp.extra_flags="-DPICO_B_MOTOR_CYCLE_HZ=20 -DPICO_B_ENABLE_OLED=1 -DPICO_B_ENABLE_MANUAL_OLED_PAGING=1 -DPICO_B_ENABLE_RELAYS=0 -DPICO_B_ENABLE_CAMERA_SERVO=0" \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE
```

B4 final build, with active-LOW relays and RDS3235 camera servo enabled:

```bash
/home/yusuf/.local/bin/arduino-cli compile --clean --warnings all \
  --fqbn rp2040:rp2040:rpipico2 \
  --libraries /home/yusuf/12_task/microros_rp2350_jazzy_port \
  --build-path /tmp/pico_b_B4_relays_rds3235_camera_servo \
  --build-property compiler.cpp.extra_flags="-DPICO_B_MOTOR_CYCLE_HZ=20 -DPICO_B_ENABLE_OLED=1 -DPICO_B_ENABLE_MANUAL_OLED_PAGING=1 -DPICO_B_ENABLE_RELAYS=1 -DPICO_B_ENABLE_CAMERA_SERVO=1" \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE
```

OLED-only regression build:

```bash
/home/yusuf/.local/bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --clean \
  --verbose \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property compiler.cpp.extra_flags=-DPICO_B_ENABLE_OLED=1 \
  --build-path /tmp/pico_b_microros_B2B_oled_build \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE
```

OLED disabled B2A regression build:

```bash
/home/yusuf/.local/bin/arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --warnings all \
  --verbose \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy \
  --build-property compiler.cpp.extra_flags=-DPICO_B_ENABLE_OLED=0 \
  --build-path /tmp/pico_b_microros_B2A_regression_build \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE
```

Expected selected library:

```text
Using library micro_ros_arduino at version 2.0.8-jazzy in folder: /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy
```

Generated UF2 files after successful compiles:

```text
/tmp/pico_b_microros_B2B_oled_build/PICO_B_MICROROS_CONTROL_NODE.ino.uf2
/tmp/pico_b_microros_B2B_regression_build/PICO_B_MICROROS_CONTROL_NODE.ino.uf2
/tmp/pico_b_microros_B3A_motor_zero_build/PICO_B_MICROROS_CONTROL_NODE.ino.uf2
/tmp/pico_b_microros_B2A_regression_build/PICO_B_MICROROS_CONTROL_NODE.ino.uf2
/tmp/pico_b_B3A_motor_10hz_oled_off/PICO_B_MICROROS_CONTROL_NODE.ino.uf2
/tmp/pico_b_B3A_motor_15hz_oled_off/PICO_B_MICROROS_CONTROL_NODE.ino.uf2
/tmp/pico_b_B3A_motor_20hz_oled_off/PICO_B_MICROROS_CONTROL_NODE.ino.uf2
/tmp/pico_b_B3_regression_no_relays_no_servo/PICO_B_MICROROS_CONTROL_NODE.ino.uf2
/tmp/pico_b_B4_relays_rds3235_camera_servo/PICO_B_MICROROS_CONTROL_NODE.ino.uf2
```

## Upload

Use the generated UF2 with the normal Pico 2 BOOTSEL mass-storage flow. Do not
upload from this build step when only static validation is intended.

## Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6
```

## Topic Checks

```bash
ros2 topic hz /pico_b/heartbeat
ros2 topic hz /pico_b/sbus/channels
ros2 topic hz /pico_b/battery/voltage
ros2 topic hz /pico_b/motors/rpm
ros2 topic hz /diagnostics
ros2 topic echo /pico_b/heartbeat std_msgs/msg/UInt32
ros2 topic echo /pico_b/sbus/channels std_msgs/msg/Int16MultiArray
ros2 topic echo /pico_b/battery/voltage std_msgs/msg/Float32
ros2 topic echo /pico_b/motors/rpm std_msgs/msg/Int16MultiArray
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray
```

Expected nominal rates:

- `/pico_b/heartbeat`: about 2 Hz, `std_msgs/msg/UInt32`, best effort.
- `/pico_b/sbus/channels`: about 20 Hz, 16 raw SBUS channels.
- `/pico_b/battery/voltage`: about 2 Hz, valid finite readings only.
- `/pico_b/motors/rpm`: about the configured motor cycle rate, order ID1 FL,
  ID2 RL, ID3 FR, ID4 RR.
- `/diagnostics`: about 2 Hz, one bounded status per publish.
  Diagnostic pages include `pico_b/sbus`, `pico_b/battery`,
  `pico_b/micro_ros`, `pico_b/oled`, `pico_b/motors`,
  `pico_b/motor_1_fl`, `pico_b/motor_2_rl`, `pico_b/motor_3_fr`,
  `pico_b/motor_4_rr`, `pico_b/motor_comm_12`, and
  `pico_b/motor_comm_34`, plus B3A audit pages
  `pico_b/motor_detail_1_fl` through `pico_b/motor_detail_4_rr`,
  `pico_b/motor_latency_12`/`pico_b/motor_latency_34`,
  `pico_b/motor_temperature_schedule`, `pico_b/motor_scheduler`,
  `pico_b/motor_transaction_guard`, `pico_b/oled_control`,
  `pico_b/relays`, and `pico_b/camera_servo`.
  `pico_b/motors` reports `motorTransactionHz` around
  `PICO_B_MOTOR_CYCLE_HZ * 4` total and `motorCycleHz` around
  `PICO_B_MOTOR_CYCLE_HZ` per motor.

## B3A Motor Audit Instrumentation

The B3A motor timing and protocol constants intentionally match the golden
`/home/yusuf/12_task/12_task.ino` motor state-machine: UART0 on GP0/GP1,
115200 `SERIAL_8N1`, CRC-8/MAXIM, 10-byte speed `0x64` frames, 10-byte
temperature `0x74` frames, 10 ms response timeout, 5 ms speed grace, 2 ms
frame quiet guard, compile-time selected motor slots, and 3 ms velocity-mode
guard. Protocol timeout values are not changed by the rate experiment.

The existing total communication counters remain on `pico_b/motor_comm_12` and
`pico_b/motor_comm_34`. Per-motor detail pages add bounded counters:
`speedTimeout`, `temperatureTimeout`, `speedCrcError`,
`temperatureCrcError`, `rxLengthLast`, and `lateResponseCount`.
`pico_b/motor_latency_12` publishes `M1_latencyLastUs`, `M1_latencyMaxUs`,
`M1_rxLengthLast`, `M2_latencyLastUs`, `M2_latencyMaxUs`, and
`M2_rxLengthLast`. `pico_b/motor_latency_34` publishes the matching M3/M4 keys.
`pico_b/motor_temperature_schedule` publishes `temperatureQueryHz`,
`M1_tempQueryCount`, `M2_tempQueryCount`, `M3_tempQueryCount`,
`M4_tempQueryCount`, and `tempScheduleMissCount`. Each diagnostics publish
still contains one status and no more than six values.

Speed commands stay at the configured compile-time motor cycle rate. Motor slot
deadlines use wrap-safe `micros()` scheduling. The 15 Hz build uses deadline
remainder accumulation so integer truncation does not create long-term drift.
`/pico_b/motors/rpm` publishes only after a fresh four-motor snapshot and never
duplicates the same snapshot sequence.

Temperature queries remain scheduled at about 1 Hz per motor with 250 ms phase
spacing: M1 at 0 ms, M2 at 250 ms, M3 at 500 ms, and M4 at 750 ms. The schedule
advances from the previous deadline and does not build an unbounded catch-up
queue after delays. If the current motor slot does not have enough room after
the speed transaction, speed scheduling has priority and
`tempScheduleMissCount` records the skipped temperature query.

`pico_b/motor_scheduler` publishes `configuredCycleHz`, `motorCycleHz`,
`motorTransactionHz`, `motorSlotPeriodUs`, `motorSlotMissCount`, and
`motorSlotLatenessMaxUs`.

## Hardware Test

1. Put all wheels fully in the air.
2. Keep people and objects clear of the motors.
3. Keep emergency power cutoff reachable.
4. Confirm the four motor IDs are wired as ID1 FL, ID2 RL, ID3 FR, ID4 RR.
5. Do not use ID-set firmware in this test.
6. Confirm there is no ROS command subscriber and the target RPM is compile-time 0.
7. Confirm GP12 and GP13 relays are OFF at first power-up.
8. Confirm the FPV camera servo does not move.
9. Confirm OLED shows the battery page, then switches to the system page about
   every 10 seconds.
10. Confirm the OLED battery page shows `PICO B   B2B`, battery voltage, SOC, and
   `STATE OK`, `LOW`, `CRITICAL`, or `INVALID`.
11. Confirm the OLED system page shows SBUS state, SBUS Hz, frame age, and ROS
   state.
12. Confirm all four motors do not move.
13. Confirm only 0 RPM plus active brake is sent to motors.
14. Start the Agent.
15. Confirm `/pico_b/heartbeat` publishes at about 2 Hz.
16. Confirm `/pico_b/sbus/channels` publishes at about 20 Hz.
17. Confirm `/pico_b/battery/voltage` publishes at about 2 Hz when ADC input is valid.
18. Confirm `/pico_b/motors/rpm` publishes at the configured motor cycle rate.
19. Confirm `/diagnostics` publishes at about 2 Hz.
20. Confirm `/pico_b/motors/rpm` order is ID1 FL, ID2 RL, ID3 FR, ID4 RR.
21. Confirm the four RPM values are 0 or within stopped-noise range.
22. Confirm an offline motor publishes 0 RPM while diagnostics reports ERROR.
23. For every motor diagnostics page, confirm `online=true`, `rpm=0`,
    `errorHex=0x00`, expected velocity mode, reasonable temperature, near-zero
    current, and CRC/timeout/wrongId counters are not continuously increasing.
24. Confirm `pico_b/motors` reports `motorTransactionHz` about
    `PICO_B_MOTOR_CYCLE_HZ * 4`, `motorCycleHz` about
    `PICO_B_MOTOR_CYCLE_HZ`, and `commandRpm=0`.
25. Confirm `pico_b/motor_temperature_schedule` reports total
    `temperatureQueryHz` about 4 Hz and each motor count rises about once per
    second.
26. Confirm `pico_b/motor_latency_12` and `pico_b/motor_latency_34` appear in
    ROS diagnostics with latency and rx-length keys.
27. Confirm `pico_b/motor_scheduler` reports the configured cycle, measured
    cycle/transaction rates, slot period, slot misses, and max lateness.
28. Run a 5 minute diagnostics capture. Confirm temperature error growth is
    clearly lower than the previous 10 Hz temperature-query test, and compare
    speed error growth with the previous 5 minute baseline.
29. For the rate experiment, flash the 15 Hz OLED-off UF2 first. Run a 5 minute
    diagnostics capture and confirm `/pico_b/motors/rpm` about 15 Hz,
    `motorCycleHz` about 15 Hz, `motorTransactionHz` about 60 Hz, `commandRpm=0`,
    four RPM values at 0, no `MOTOR_OFFLINE`/`MOTOR_FAULT`, wrong ID 0, late
    response 0, latency max below 10000 us, RX length 10, and scheduler
    miss/lateness reported.
30. If 15 Hz passes, flash the 20 Hz OLED-off UF2. Run 5 minutes, then 10
    minutes, and confirm `/pico_b/motors/rpm` about 20 Hz, `motorCycleHz` about
    20 Hz, and `motorTransactionHz` about 80 Hz with the same safety checks.
    Calculate the normalized error rate for each run.
31. Turn the transmitter off. Confirm SBUS safe-neutral `1002`, motor target
    remains 0, and motors do not move.
32. Stop the Agent with Ctrl+C. Confirm the local motor task continues 0 RPM plus
    brake and motors do not move.
33. Restart the Agent. Confirm Pico reconnects without reset and motor
    topic/diagnostics return.
34. With motor power off, disconnect one motor communication line, then power up.
    Confirm that motor is offline, diagnostics reports `MOTOR_OFFLINE`/ERROR,
    the other motor targets remain 0, and no motor moves.
35. Run 10 minutes, then 30 minutes. Confirm publish errors stay 0, unexpected
    reconnect stays 0, USB disconnect does not occur, CRC/timeout/wrongId do not
    continuously increase, all four motors stay online with RPM 0, temperatures
    stay safe, and OLED/SBUS/battery frequencies keep the B2B values.

## B3A 20 Hz basic OLED + CH3 manual paging PENDING hardware criteria

Build flags: `-DPICO_B_MOTOR_CYCLE_HZ=20 -DPICO_B_ENABLE_OLED=1
-DPICO_B_ENABLE_SPLASH_UI=0 -DPICO_B_ENABLE_MANUAL_OLED_PAGING=1`

Build path: `/tmp/pico_b_B3A_20hz_oled_ch3_manual/`
UF2 SHA256: `8207413c6df9cb9f33ef968dbfeb8ec2a0e6813247fb8535cb246ee63b563124`

PICO_B source backup (post-build, read-only):
`/home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE_BACKUP_B3A_CH3_MANUAL_POSTBUILD_20260901_084617/`

**Source baseline restored from PASS hardware backup:**
B3A 20 Hz PASS backup path:
`/home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE_BACKUP_B3A_20HZ_PASS_20260901_070423/`
with pre-CH3 baseline `pico_b_node_app.cpp` SHA256
`03a4399c0c4e533f12bdb24bb98385617230b6d956bcf050c9713f6b5e34dcd2`.
After adding CH3 manual paging only (no splash, no 1 MHz, no starvation
counters, no deadline-aware flush, no core1, no partial transfers) the
final source hash is
`9ab4adf82be6af5a29332dd316a3196f241447e4fb1710121dfb9b49ca0330c8`.

Hardware criteria list (user-ordered; run each item and PASS/FAIL explicitly):

1. Açılışta temel OLED normal çalışmalı — güç verdikten sonra BATTERY veya
   SYSTEM sayfası görüntülenmeli.
2. Ekran splash'ta kalmamalı — B3A baseline'te splash state machine ve
   "LESSIEN/By Yusuf Karakus/wink" OLMADIĞI için açılışta doğrudan temel
   sayfa görünmeli.
3. Otomatik sayfa geçmemeli — 10 s veya sabit süreli otomatik sayfa döngüsü
   KALDIRILMIŞTIR (yalnız CH3 manuel build); sayfalar yalnız CH3 hareketiyle
   değişmeli.
4. CH3 neutral → up (CH3 >= 1300): YALNIZ TEK SAYFA İLERİ. Ardından kol tekrar
   neutral (900-1100) aralığına dönmeden ikinci bir ileri geçiş OLMAMALI.
5. CH3 neutral → down (CH3 <= 700): YALNIZ TEK SAYFA GERİ. Ardından kol tekrar
   neutral aralığına dönmeden ikinci bir geri geçiş OLMAMALI.
6. Basılı tutma tekrar oluşturmamalı — kol yukarıda veya aşağıda sürekli
   tutulduğunda sayfa sürekli değişmemeli, yalnızca neutral → up/down
   KENAR (edge) geçişlerinde TEK sefer değişmeli.
7. Kumanda kapalıyken (transmitter off, failsafe, SBUS timeout, frame lost,
   veya bağlantı kopması durumunda) sayfa DEĞİŞMEMELİ ve input arm
   durumunda SIFIRLANMALI (sentetik safe-neutral 1002 sistemi arm ETMEMELİ).
8. Motor RPM ~20 Hz ve dört RPM değeri [0, 0, 0, 0] olmalı; MOTOR_OFFLINE ve
   MOTOR_FAULT OLMAMALI; `temperatureQueryHz` 3.8–4.2 Hz aralığında olmalı.
9. micro-ROS Agent yeniden bağlandığında (reconnect) OLED sayfası
   DEĞİŞMEMELİ; Agent bağlantısı koptuğunda/geri geldiğinde dahili motor
   döngüsü 0 RPM + brake 0xFF devam etmeli.

Compile/static evidence scope for this build variant (see TEST_RESULTS.md 7th
entry for full chain; hardware items above are SEPARATE pass/fail):

- arduino-cli `--warnings all` 0 warnings / 0 errors (BOTH regression and CH3
  manual builds).
- cppcheck 0 PICO_B source issues.
- Golden `12_task.ino` SHA256 preserved:
  `404b81d8f63ef26ab1e4ca2371cf5eb0960280b2e3ea67aaf34732d7a6d024fa`.
- 4 strong `arduino_transport_*` ELF T symbols present;
  `arduino_transport_close()` NO `Serial.end()` call.
- Publisher count = 5 (heartbeat, sbus, battery, motors_rpm, diagnostics);
  0 subscribers, 0 services, 0 clients. New `pico_b/oled_control` bounded
  diagnostic page is INSERTED INTO the existing `/diagnostics` rotation
  (publisher count UNCHANGED).
- Motor safety: RPM 0 static assert, brake 0xFF static assert, UART0 115200
  8N1, CRC-8/MAXIM (poly 0x8c), temperature 1 Hz/motor (phased), motor IDs
  [1=FL, 2=RL, 3=FR, 4=RR] static assert, GP12/GP13 boot HIGH/OFF,
  PICO_B_ENABLE_SERVO=0, PICO_B_ENABLE_SPI_BRIDGE=0, NO cmd_vel subscriber,
  NO ID setter/broadcast strings in final ELF.
- OLED I2C 400 kHz baseline preserved (setupOled `Wire.setClock(400000)`).
  NO 1 MHz clock, NO deadline-aware flush, NO starvation escape, NO
  oledFlushStarvationCount, NO core1 OLED, NO partial display transfer.
- CH3 paging state machine behavior (verified at source-level):
  CH3 = `sbusStatus.channels[2]` (index 2 / third channel), neutral 900–1100
  ARMS input; >= 1300 advance + disarm; <= 700 back + disarm; unhealthy
  SBUS (disconnected / timeout / failsafe / frame-lost / synthetic 1002)
  DISARMS and increments ignored counter; pages wrap-around modulo
  `OledPage::COUNT` sentinel.

## B3A 20 Hz basic OLED + CH3 manual paging + MOTORS OLED page PENDING hardware criteria

Build flags: `-DPICO_B_MOTOR_CYCLE_HZ=20 -DPICO_B_ENABLE_OLED=1
-DPICO_B_ENABLE_SPLASH_UI=0 -DPICO_B_ENABLE_MANUAL_OLED_PAGING=1`

Build path (user-specified exact new directory):
`/tmp/pico_b_B3A_20hz_oled_ch3_motor_page/`
UF2 SHA256: `f84ee4b9475b22adee31efabed47463eeb3e266ba2a4a170766dc41a65c789e2`
UF2 boyutu: 349.696 bayt
PICO_B son kaynak SHA256 (MOTORS sayfası eklenmiş):
`3db6dae5f3e58aa37ba42d20ab8687ef2e7d70ea61119edca08f1a0903a6c1f9`

Pre-edit (sadece-CH3, MOTORS olmadan) READ-ONLY backup path:
`/home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE_BACKUP_CH3_MANUAL_PRE_MOTORPAGE_20260901_090740/`
with internal `pico_b_node_app.cpp` SHA256
`9ab4adf82be6af5a29332dd316a3196f241447e4fb1710121dfb9b49ca0330c8`.

New OLED page order after MOTORS page addition:
1. BATTERY (pageIndex 0)
2. SYSTEM (pageIndex 1)
3. MOTORS (pageIndex 2 — NEW)

MOTORS page layout, exactly as drawn:
```
MOTORS
M1 FL <STATE> <RPM>
M2 RL <STATE> <RPM>
M3 FR <STATE> <RPM>
M4 RR <STATE> <RPM>
```

Motor mapping preserved (static_assert verified): ID1 = FL, ID2 = RL,
ID3 = FR, ID4 = RR. MOTORS page reads ONLY existing runtime motor fields:
`motorStatus[i].online`, `motorStatus[i].lastValidMs`,
`motorStatus[i].errorCode`, `motorStatus[i].measuredRpm`, and the existing
helper `motorTelemetryFresh(i, nowMs)`. MOTORS page write-path:
`printOledLine(...)` calls bounded `snprintf` into stack `char line[24]`
(NO Arduino `String`, NO `new`/`malloc`, NO counter writes, NO motor
scheduler/temperature or health state machine MODIFICATIONS).

STATE rendering rules (read-only, no writes to motor counters):
- `OK `  → motor telemetry FRESH (online + lastValid <= timeout) AND
          `errorCode == 0x00`.
- `ERR` → motor telemetry FRESH AND `errorCode != 0x00` (motor-side fault
          flag exists).
- `OFF` → motor offline OR stale telemetry OR valid telemetry never received
          (state machine regards motor as non-communicating).

RPM field shows the TELEMETRY feedback value `measuredRpm` from the DDSM115
response, NOT the compile-time target-RPM 0 constant. If telemetry is not
fresh, RPM is forced to 0 in the rendered line (read-only clamp).

MOTORS page Turkish hardware criteria list (user-ordered; PASS/FAIL each):
1. CH3 kumanda kolu ile MOTORS sayfasına GEÇİLEBİLMELİ — BATTERY/SYSTEM
   sayfalarındayken neutral → CH3 up (>= 1300) 2 adet TEK kenar geçişi
   yapıldığında MOTORS sayfasına gelmeli; pageName /diagnostics içinde
   "MOTORS" olarak görünmeli; geri dönüşte neutral → down 2 kere SYSTEM
   → BATTERY sırasında wrap-around olmalı.
2. Dört DDSM115 motor DAİMA bağlıyken M1 FL, M2 RL, M3 FR, M4 RR HEPSİ
   `OK ` olarak görünmeli; hiçbir satır OFF/ERR olmamalı (hava kablosu
   takılı, kumanda açık, Agent bağlı veya değil fark etmez).
3. MOTORS sayfasında görüntülenen measured RPM dört motor da 0 (sıfır)
   olarak görünmeli. Hedef RPM sabit 0 (compile-time) olduğundan telemetri
   değeri 0 RPM olmalı (tork akışı yok, ancak haberleşme sağlıklı).
4. Test amacıyla TEK bir motorun RS485 kablosu (VCC/GND hariç A/B hattı)
   ÇEKİLDİĞİNDE — YALNIZ İLGİLİ motor satırı `OFF` olarak görünmeli;
   DİĞER ÜÇ motor satırı hala `OK ` kalmalı (global reset olmamalı).
5. Aynı motor kablosu TEKRAR takıldığında — mevcut motor state machine
   (ardışık başarılı cevaplar ve consecutiveFailureCount sıfırlama izni
   verdiğinde) ilgili motor satırı TEKRAR `OK ` a DÖNMELİ.
6. Motor error byte != 0x00 (örneğin gerçek DDSM115 fault byte) geldiyse
   ilgili motor satırı `ERR` olarak görünmeli; ERR görünmesine rağmen
   diğer state machine sayaçları ve güvenlik değerleri (RPM=0, brake 0xFF)
   DEĞİŞMEMELİ.
7. Tüm bu ekran işlemleri ve manual page switch sırasında motor döngüsü
   yine ~20 Hz ve temperature query toplam yine ~3.8–4.2 Hz korunmalı
   (ekran çizimi motor slotunu engellememeli; motorAllowsSlowTasks guard
   çalışıyor; display.display() deadline-aware scheduler yok).

Source change from pre-MOTORS hash was MINIMAL and ONLY in OLED rendering
layer (4 code edits: enum insert, oledPageName case, switch MOTORS case,
new `drawMotorsOledPage()` function). Motor scheduler, UART/CRC/timeout/
opcode frame, reconnect, SBUS parser/scaling, ROS topics, diagnostics
publisher count, OLED 400 kHz I2C clock, display.display() baseline flow
were NOT modified (see AGENTS.md DO-NOT-CHANGE list).

## B3A 20 Hz basic OLED + CH3 manual paging + MOTORS OLED page + TEMPERATURE PENDING schedule — PENDING hardware criteria (temp pending variant)

User-explicit temperature pending fix (B3A 20 Hz + CH3 manual + MOTORS page +
MINIMAL bounded temperature pending/overdue state machine, 8 code edits ONLY
in temperature scheduler layer, NO OLED/I2C/20Hz/slot/CRC/timeout change).

**Amaç:** M1/M3 slotlarının OLED flush (~28.6 ms, 2 Hz) ile phased çakışması
yüzünden `temperatureQueryHz=2.0` hedefin yarısı kalmasın; pending carry ile
kaçırılan sıcaklık sorgularını sonraki motor slotunda göndererek toplam
`3.8–4.2 Hz` (motor başına ~1 Hz) hedefi.

### Pre-edit & code-change chain (STOP-on-mismatch evidence)

1. PRE-TEMPERATURE-PENDING FIX çalışan ağaç SHA: `pico_b_node_app.cpp` =
   `3db6dae5f3e58aa37ba42d20ab8687ef2e7d70ea61119edca08f1a0903a6c1f9`
   (CH3 manual + MOTORS sayfası, session 2 final; TEST_RESULTS #8).
   Pre-edit hash MATCH ✅ doğrulandı; golden `12_task.ino` da aynı
   `404b81d8…24fa` korunuyor.
2. PRE-TEMPERATURE-PENDING FIX tarihli salt-okunur yedek:
   `/home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE_BACKUP_PRE_TEMP_PENDING_20260901_095538/`
   (iç yedek pico hash = `3db6dae5…c1f9` MATCH; `chmod -R a-w` donmuş kanıt).
3. Sonrasında YALNIZ 8 MINIMAL temperature scheduler katmanı değişikliği
   (aşağıdaki code-change proof tablosu) uygulandı.
4. Post-temperature-pending final `pico_b_node_app.cpp` SHA256:
   `8dd5fa5c16a5238583cb47b4f79c970f86a9c9c03606ea4fbaddf0736a04b0ba`.

### Code-change proof (8 minimal temperature scheduler edits ONLY; NO OLED/20Hz/I2C/slot/CRC changed)

| # | Location | Change | Guard check |
|---|----------|--------|-------------|
| 1 | L168 (constexpr) | `DIAG_VALUE_CAPACITY` 6 → 7 (tek bounded değer kapasitesi artışı). | Diagnostics hala status.size=1; values.size ≤7; NO new publisher. |
| 2 | L392-L399 (globals) | Bounded 4-bit `uint8_t motorTemperaturePendingMask` + `bool temperatureQueryThisSlotStarted` eklendi. | 4 bit = 1 bit/motor bounded; heap yok. Kural 2 (pending mask) + kural 7 (slot cap). |
| 3 | L1083 advanceMotorTemperatureDeadline | MissCount++ KALDIRILDI (deadline advance YALNIZ başarılı SEND sonrası çalışır). | Başarılı gönderim sonrası 1sn atlama doğru; başarısız beklemede skip yok. |
| 4 | L1095 skipMotorTemperatureDeadline | TÜM DAVRANIŞ YENİDEN: eski (miss++ + deadline atla 1sn) yerine YENİ (sadece `pendingMask |= (1<<motorIndex)`). Deadline artırma YOK, miss sayma YOK burda. | Kural 1 (düşürme), 4 (beklet), 12 (her döngü tekrar sayma) sağlar. |
| 5 | L1121 finishSpeedTransaction | Eski (sadece due+window varsa gönder, yoksa SKIP+DROP) yerine YENİ `hasPendingOrDue` = due VEYA pending mask. Uygunsa gönder (pending temizle + slotStarted=true); değilse pending set. NEXT_MOTOR'a düş. | Kural 3 (speed sonrası uygun window), 6 (pending mask ile duplicate/burst engel), 8 (speed her zaman 1. öncelik). |
| 6 | L855 sendMotorTemperatureQuery + L1176 init + L1279 NEXT_MOTOR | `sendMotorTemperatureQuery`: savunma amaçlı tekrar pending mask temizle + slot cap set. `initializeMotorTask`: pending mask 0 / slot cap false boot sıfırlama. NEXT_MOTOR (L1279-L1283): slot bitişinde EĞER pending hala SET ise missCount++ TEK KEZ (kural 12); sonra slotStarted reset (kural 7). | Kural 5 (başarıyla gönderilince 1sn deadline advance zaten mevcut burada çağrılıyor), kural 7 (aynı slot max 1 sorgu), kural 12 missCount semantiği. |
| 7 | L2032 fillMotorTemperatureScheduleDiagnostic | 7. KeyValue `temperaturePendingMask` hex 0x%02X olarak eklendi (L2050 setDiagnosticValueHex8). | Kural 13: bounded diagnostics, Mevcut /diagnostics rotation içinde (publisher 5 AYNI), 7 KeyValue. |

### Build result (FINAL user-explicit path; 0W0E --warnings all)

| Item | Value |
|------|-------|
| Build flags | `PICO_B_MOTOR_CYCLE_HZ=20 PICO_B_ENABLE_OLED=1 PICO_B_ENABLE_SPLASH_UI=0 PICO_B_ENABLE_MANUAL_OLED_PAGING=1` |
| Path (user-explicit required directory) | `/tmp/pico_b_B3A_20hz_oled_ch3_motor_page_temp_pending/` |
| Exit code | 0 |
| Warnings | 0 |
| Errors | 0 |
| Program | **146260 bytes (3%)** (+136 B vs CH3+MOTORS regression; sıcaklık pending state machine + diagnostics slot) |
| Global variables | **35780 bytes (6%)** |
| UF2 SHA256 (FINAL, user deliverable; flash ÖNCE tekrar doğrula) | **`cc5769ada484f29bffd71063e8fe8e890d8cc9c02e8c1b367b10b6406bd06932`** |
| UF2 size | **349696 bytes** |

### 10-maddelik DONANIM test kriterleri (user-ordered explicit list; flash sonrası 5 dk + gözlem)

Aşağıdaki 10 kriterin HEPSİ donanımda gözlemlenerek PASS/FAIL olarak
TEST_RESULTS.md yeni girdisine APPEND ONLY olarak kaydedilmeli (mevcut
girdilere GERİYE DOKUNULMAYACAK). Şu an compile/static kanıtı mevcut;
HW PASS iddiası YOK.

1. **RPM [0,0,0,0]** — `/pico_b/motors/rpm` topic echo çıktısı her motor
   için 0 RPM göstermeli (target RPM compile-time 0 sabit, hareket YOK).
2. **motors/rpm 19.5–20.5 Hz** — `ros2 topic hz /pico_b/motors/rpm` bu
   bant içinde kalmalı.
3. **motorCycleHz 19.5–20.5** — diagnostics scheduler sayfası bu aralıkta.
4. **motorTransactionHz 78–82** — dört motor × 20 Hz = 80 nominal.
5. **temperatureQueryHz 3.8–4.2** — hedef (motor başına yaklaşık 1 Hz,
   toplam 4 Hz; eski 2.0 Hz yarısı düzelmiş olmalı).
6. **Dört tempQueryCount benzer hızla artmalı** — M1..M4 dördü de 5 dk
   sonunda ~280-320 arası yakın sayılar göstermeli (tek motor 0 veya
   tek motor 2× others OLMAYACAK; eski donanım testinde M1=1, M3=0
   duzeltildi göstergesi).
7. **Pending mask sürekli takılı kalmamalı** — diagnostics
   `temperaturePendingMask` 0x0, 0x1, 0x4, 0x5 vs geçici değerler alsın;
   5 dakika sonunda sürekli 0xF veya sabit tek bit OLMAYACAK (sorgular
   gönderilip pending mask temizlenmeli).
8. **motorSlotMissCount artmamalı** — 5 dk başı ~0-5 arası normal;
   monoton sürekli artış OLMAYACAK (motor slotunu OLED/temp işlemleri
   kaçırmıyor).
9. **MOTOR_OFFLINE/MOTOR_FAULT olmamalı** — 5 dk yakalama süresince
   onlineMask 0x0F, faultMask 0x00 kalmalı.
10. **CH3 sayfa kontrolü BOZULMAMALI** — BATTERY↔SYSTEM↔MOTORS üç sayfa
    wrap-around neutral-armed edge trigger çalışmaya devam etmeli;
    sayfa değişimleri pending mask veya sıcaklık kodundan etkilenmemeli.

## B3A 20 Hz + CH3 manual + MOTORS OLED + temperature guard follow-up

Follow-up source hash after the transient OFF/ON and RS485 error audit:
`pico_b_node_app.cpp`
`1e904677cfaafc848013dda4689e4cd1b06865af6946dd59e4ac5664cc4e118b`.

Pre-fix read-only backup:
`/home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE_BACKUP_PRE_TEMP_GUARD_20260901_104310/`
with `pico_b_node_app.cpp`
`8dd5fa5c16a5238583cb47b4f79c970f86a9c9c03606ea4fbaddf0736a04b0ba`.

Log analyzed:
`/tmp/pico_b_temp_pending_5min.txt`
SHA256 `025d9ae73890cfa722e1f4ac2c09fb2b4e8c4c32598f853f26c9c955b18df260`.

Evidence from that log:

- `temperatureQueryHz` reached `4.0`.
- M1..M4 temperature query counts were balanced at the end.
- `motorCycleHz=20.0`, `motorTransactionHz=80.0`.
- `motorSlotMissCount` stayed `84`.
- Final `onlineMask=0x0F`, `faultMask=0x00`, `commandRpm=0`.
- Speed CRC/timeout counters still increased.
- User observed transient M2 OFF/ON on the MOTORS OLED page.

Extra log analyzed:
`/tmp/pico_b_m2_off_on_check.txt`
SHA256 `003a1b45d4f05285a26cf2542e7e3138ee565eb3a5f7644da9089cdfa616f7dc`.

Extra evidence:

- `pico_b/motor_2_rl` was `online=true` in 12/12 sampled diagnostics pages.
- `pico_b/motors` reported `onlineMask=0x0F` in 12/12 sampled pages.
- No `MOTOR_OFFLINE` or `MOTOR_FAULT` appeared in diagnostics.
- Treat the OLED M2 OFF/ON symptom as a display/render consistency issue unless
  a diagnostics log proves real offline.

Root cause fixed in this follow-up:

- Temperature CRC/timeout errors no longer increment the same
  `consecutiveFailureCount` used for speed-health offline decisions.
- Temperature responses no longer refresh `lastValidMs`; speed telemetry alone
  owns motor online freshness.
- `DIAG_VALUE_CAPACITY` is back to 6, so every diagnostics page remains
  `values.size<=6`.
- `temperaturePendingMask` was removed from the 7th value slot and replaced by a
  separate bounded diagnostics page:
  `pico_b/motor_transaction_guard`.
- Temperature start is rejected if the speed slot already started late, and the
  next-speed-deadline guard now includes TX serialization, measured response
  budget, quiet period, and margin.
- MOTORS OLED page now takes a bounded four-motor local snapshot at frame start.
  It shows `OFF` only from snapshot `online=false`, `ERR` only from
  `online=true && errorCode!=0`, and `OK ` only from
  `online=true && errorCode==0`.

New diagnostics page:

`pico_b/motor_transaction_guard`

- `tempStartCount`
- `tempDeferredCount`
- `tempGuardRejectCount`
- `tempStartLatenessMaxUs`
- `speedConsecutiveFailMax`
- `lastOfflineMotorId`

Final build:

```bash
/home/yusuf/.local/bin/arduino-cli compile --clean --warnings all \
  --fqbn rp2040:rp2040:rpipico2 \
  --libraries /home/yusuf/12_task/microros_rp2350_jazzy_port \
  --build-path /tmp/pico_b_B3A_20hz_oled_ch3_motor_page_temp_guard \
  --build-property compiler.cpp.extra_flags="-DPICO_B_MOTOR_CYCLE_HZ=20 -DPICO_B_ENABLE_OLED=1 -DPICO_B_ENABLE_MANUAL_OLED_PAGING=1" \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE
```

Final UF2:
`/tmp/pico_b_B3A_20hz_oled_ch3_motor_page_temp_guard/PICO_B_MICROROS_CONTROL_NODE.ino.uf2`

UF2 SHA256:
`e3421f10da4c5354cc0514a7078e4e8458fac52f1abd72435d2f0b485426b5bb`

Build result:

- Program: 146724 bytes.
- Global variables: 35756 bytes.
- `--warnings all`: PASS.
- cppcheck: PASS.
- Hardware PASS: NOT VERIFIED.

Hardware test criteria for this follow-up:

1. RPM `[0,0,0,0]`.
2. `/pico_b/motors/rpm` 19.5-20.5 Hz.
3. `motorCycleHz` 19.5-20.5.
4. `motorTransactionHz` 78-82.
5. `temperatureQueryHz` 3.8-4.2.
6. M1..M4 `tempQueryCount` values rise evenly.
7. `pico_b/motor_transaction_guard` appears in `/diagnostics`.
8. `tempGuardRejectCount` may rise when a temp query would be unsafe; this must
   not reduce `temperatureQueryHz` below 3.8-4.2 over the full run.
9. `motorSlotMissCount` must not increase after startup.
10. `onlineMask` must remain `0x0F`; `faultMask` must remain `0x00`.
11. MOTORS page must not show M1-M4 OFF/ON flicker during normal operation when
    diagnostics keep `online=true` and `onlineMask=0x0F`.
12. Speed CRC/timeout growth must be clearly lower than
    `/tmp/pico_b_temp_pending_5min.txt`.
13. CH3 manual paging must still work.

## B4 Relay + RDS3235 Camera Servo Hardware Test Plan

This plan is pending hardware execution; do not record PASS without a real log.

Preparation:

- First test with wheels lifted.
- The RDS3235 servo may be mechanically disconnected from the camera load for
  the first motion check.
- Confirm external servo BEC is 6-7.4 V and shares GND with Pico.
- Confirm relay wiring expects active LOW outputs.

Expected boot behavior:

- GP12 and GP13 are latched HIGH/OFF before `pinMode(OUTPUT)`.
- The first physical servo command after attach is exactly `1570 us` center.
- No automatic sweep, endpoint search, or calibration motion occurs.

Runtime checks:

- Relay 1 follows SBUS CH7 and relay 2 follows SBUS CH8.
- Relay ON threshold is `>=1400`; OFF threshold is `<=600`.
- Relay ON requires a real OFF/re-arm state and `100 ms` debounce.
- SBUS disconnect, timeout, frame-lost, or failsafe forces both relays OFF
  within the SBUS timeout behavior.
- micro-ROS Agent stop/restart does not stop local SBUS failsafe tasks.
- Servo uses SBUS CH4, 20 ms cooperative updates, and 90 deg/s slew.
- Servo unhealthy-input failsafe returns target to center using the same
  cooperative slew path.
- Servo pulse stays within `550-2450 us`; calibration stays
  `550/1570/2450 us`.
- Motors remain `[0,0,0,0]`, about 20 Hz cycle, about 80 Hz total transaction.
- `temperatureQueryHz` stays 3.8-4.2.
- `onlineMask=0x0F`, `faultMask=0x00`; no `MOTOR_OFFLINE` or `MOTOR_FAULT`.
- Existing OLED BATTERY/SYSTEM/MOTORS pages and CH3 manual paging still work.

Diagnostics pages to verify in `/diagnostics`:

- `pico_b/relays`: `relay1On`, `relay2On`, `relay1Requested`,
  `relay2Requested`, `outputSafe`, `relayFailsafeCount`.
- `pico_b/camera_servo`: `attached`, `targetAngleDeg`, `currentAngleDeg`,
  `pulseUs`, `inputHealthy`, `servoFailsafeCount`.

## B5A Build (2026-09-01, /cmd_vel receive-only subscriber)

**IMPORTANT LIBRARY RULE for B5A:**
The Jazzy micro-ROS port contains two separate static library archives. Do NOT
modify the original Pico A SUB=0 archive at
`/home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy/`
(lib SHA `34930364ef68e906575300d34b8aa067a280fb4e6c12d845d39dbc582abd1277`).
For B5A builds use the new SEPARATE Pico-B-only SUB=1 archive at
`/home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy_pico_b/`
(lib SHA `e62ed07fd8accc84367c21ec8241caf6bcd05857afa9d1feba810327a95ce099`,
RMW: PUB=10 SUB=1 SVC=0 CLNT=0 HIST=2 CUSTOM TRANSPORT, geometry_msgs/Twist
typesupport included).

Because both libraries coexist in the parent port directory and `arduino-cli`
selects by the `name=` field in `library.properties` (which must match
`<micro_ros_arduino.h>`), you must TEMPORARILY swap libraries when building
B5A. After building restore the default so Pico A builds continue to select
the original SUB=0 library. The permanent `pico_b` folder stores
`name=micro_ros_arduino_pico_b` so it is skipped by default unless explicitly
swapped.

### B5A B4 regression build (CMD_VEL=0 → baseline B4 behavior unchanged):

```bash
# Uses original Pico A SUB=0 library (default library name in place; no swap)
arduino-cli compile --clean --warnings all \
  --fqbn rp2040:rp2040:rpipico2 \
  --libraries /home/yusuf/12_task/microros_rp2350_jazzy_port \
  --build-path /tmp/pico_b_B4_regression_cmd_vel_disabled \
  --build-property compiler.cpp.extra_flags="-DPICO_B_MOTOR_CYCLE_HZ=20 -DPICO_B_ENABLE_OLED=1 -DPICO_B_ENABLE_MANUAL_OLED_PAGING=1 -DPICO_B_ENABLE_RELAYS=1 -DPICO_B_ENABLE_CAMERA_SERVO=1 -DPICO_B_ENABLE_CMD_VEL_SUBSCRIBER=0" \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE
```
B4 regression UF2 SHA256: `a0a6fdc2a1d8166f092ed3f1524c0128da5e544b3edebe916e1d6a99c3546993`.
Build produces EXIT 0 with 0 warnings/0 errors; subscriber path is not linked
(dead code guarded by `#if PICO_B_ENABLE_CMD_VEL_SUBSCRIBER == 0`).

### B5A FINAL build (CMD_VEL=1 → new separate SUB=1 Pico B library):

Library swap recipe BEFORE the final build (restore after build completes):

```bash
PORT=/home/yusuf/12_task/microros_rp2350_jazzy_port
PICO_A=${PORT}/micro_ros_arduino_rp2350_jazzy
PICO_B=${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b
# (1) Move Pico A out of the port scan path temporarily so it cannot be selected
mv "${PICO_A}" /tmp/pico_a_jazzy_tmp_for_b5a_build
# (2) Swap Pico B library.properties to the header-expected name AND make sure
#     the precompiled copy exists both at src/cortex-m33 AND src/cortex-m33/softfp
#     (arduino-cli precompiled searches src/cortex-m33, NOT only softfp subdir)
cp -n "${PICO_B}/src/cortex-m33/softfp/libmicroros.a" "${PICO_B}/src/cortex-m33/libmicroros.a" 2>/dev/null || true
sed -i 's|^name=.*|name=micro_ros_arduino|' "${PICO_B}/library.properties"
```

Now run the B5A final build using the swapped library (the port directory now
contains ONLY the Pico B archive with the matching library name):

```bash
arduino-cli compile --clean --warnings all \
  --fqbn rp2040:rp2040:rpipico2 \
  --libraries /home/yusuf/12_task/microros_rp2350_jazzy_port \
  --build-path /tmp/pico_b_B5A_cmd_vel_rx_only \
  --build-property compiler.cpp.extra_flags="-DPICO_B_MOTOR_CYCLE_HZ=20 -DPICO_B_ENABLE_OLED=1 -DPICO_B_ENABLE_MANUAL_OLED_PAGING=1 -DPICO_B_ENABLE_RELAYS=1 -DPICO_B_ENABLE_CAMERA_SERVO=1 -DPICO_B_ENABLE_CMD_VEL_SUBSCRIBER=1" \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE
```
B5A FINAL UF2 (EXACT user-required path):
`/tmp/pico_b_B5A_cmd_vel_rx_only/PICO_B_MICROROS_CONTROL_NODE.ino.uf2`
UF2 SHA256: **`75c0acf5a9358f57e96512b343b430d0c08354a4026e8469b1b7a4211bc2ee17`**
EXIT 0 with 0 warnings/0 errors; program=152124 bytes; globals=36228 bytes.

Library swap RESTORE after the build (so default port scan goes back to Pico A):

```bash
PORT=/home/yusuf/12_task/microros_rp2350_jazzy_port
PICO_A=${PORT}/micro_ros_arduino_rp2350_jazzy
PICO_B=${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b
# Restore Pico B label to its permanent distinct name so it cannot be selected
# accidentally by a Pico A build that passes --libraries <PORT> without swap.
sed -i 's|^name=.*|name=micro_ros_arduino_pico_b|' "${PICO_B}/library.properties"
# Restore Pico A to the port parent path
mv /tmp/pico_a_jazzy_tmp_for_b5a_build "${PICO_A}"
```

### B5A feature invariants (compile-verified, runtime hardware-gated):

- QoS: `/cmd_vel` → BEST_EFFORT reliability, VOLATILE durability,
  KEEP_LAST history depth=1 (compatible with default `ros2 topic pub`).
- Executor handle count = EXACTLY 1.
- Callback is allocation-free and writes only a bounded CmdVelStatus snapshot
  (primitive fields only). NaN/Inf → rejected (rejectCount++ + invalid flag);
  finite valid → bounded snapshot update only.
- Callback NEVER writes motor target tables, never calls RS485 TX, never
  publishes, never delays or logs. Motor RPM stays compile-time 0; brake
  stays `0xFF` static_assert verified inside the subscriber guard block.
- Receipt freshness uses LOCAL `millis()` wrap-safe age; no ROS time sync
  needed. Fresh timeout: `CMD_VEL_FRESH_TIMEOUT_MS = 250` ms.
- Snapshot is invalidated at boot (`setup()` after `zeroEntities()`) and on
  every `destroyEntities()` (agent drop / reconnect cycle).
- Diagnostics page `pico_b/cmd_vel` is INSERTED INTO the existing
  `/diagnostics` publisher rotation. Publisher count stays 5; NO new publisher
  is created. Exactly 6 KeyValues + bounded level/message. NO Arduino
  `String`/`new`/`malloc`; bounded static `snprintf` buffer only.
- Reconnect state machine (WAIT_AGENT/CREATE_ENTITIES/RUNNING/DESTROY_ENTITIES)
  preserved. Local tasks (SBUS, relays, servo, 20 Hz motor slots with 80 Hz
  transaction cadence, 2 Hz OLED 400 kHz CH3-manual paging, battery ADC)
  all run BEFORE bounded nonblocking `rclc_executor_spin_some(&exec, 0)`
  inside RUNNING. Destroy cleanup order (reverse): sub fini → guarded exec
  fini → zero handles → snapshot invalidate → existing publisher/node/support
  fini. Destroy timeout stays 0.
- Strong custom transport symbols preserved; `arduino_transport_close` still
  `return true;` with NO call to `Serial.end`.
- NO ID setter / broadcast opcode. NO SPI bridge init.

## B5A Hardware Test Plan (14 kriter, USER-ordered explicit list)

Preparation: Wheels fully air-gapped. Emergency power cutoff reachable. Motor
power cables connected but wheel load physically disconnected. Flash UF2 above
only after verifying 6-file hash chain matches AGENT_HANDOFF.md.

Test command prerequisites:
```bash
# Pre-flash STOP-on-mismatch:
sha256sum /home/yusuf/12_task/12_task.ino
# expected: 404b81d8f63ef26ab1e4ca2371cf5eb0960280b2e3ea67aaf34732d7a6d024fa
sha256sum /tmp/pico_b_B5A_cmd_vel_rx_only/PICO_B_MICROROS_CONTROL_NODE.ino.uf2
# expected: 75c0acf5a9358f57e96512b343b430d0c08354a4026e8469b1b7a4211bc2ee17
```

1. **Tekerlekler havada** (all four motor wheels air-gapped, no ground contact).
2. **Motorlar her koşulda [0,0,0,0]** — `/pico_b/motors/rpm` 4-element array
   stays all-zero measured RPM; `commandRpm` diagnostics key stays 0;
   `MOTOR_TARGET_RPM` compile-time 0 preserved; brake `0xFF`.
3. `/cmd_vel` publish OLMADAN önce diagnostics `pico_b/cmd_vel` page shows:
   level = WARN, message = `CMD_VEL_NEVER_RECEIVED`; `receiveCount = 0`;
   `rejectCount = 0`; `commandFresh = false`; `linearX = 0`; `angularZ = 0`.
4. Start the verbatim 10 Hz test publisher:
   ```bash
   ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.10}, angular: {z: 0.20}}"
   ```
5. Publisher çalışırken `pico_b/cmd_vel` diagnostics report:
   `linearX ≈ 0.10` (2 decimal places), `angularZ ≈ 0.20`, `receiveCount`
   monotonically artıyor, `commandFresh = true`, `commandAgeMs < 250`,
   level = OK, message = `CMD_VEL_RX_ONLY`.
6. Yayını DURDUR (Ctrl+C on the publisher). After ≥ 250 ms of silence the
   same diagnostics page transitions to level = WARN / message =
   `CMD_VEL_STALE`; `commandFresh = false`; last `linearX`/`angularZ` still
   show the last valid snapshot but age grows past 250 ms wrap-safe.
7. Yeniden çalıştır pozitif, negatif, ve sıfır `linear.x` + `angular.z`
   değerleri (örn. x=+0.50 z=-0.30; x=-0.20 z=+0.05; x=0 z=0; her biri için
   en az 10 yayın yap). Bütün bu varyasyonlar boyunca dört motor measured
   RPM [0,0,0,0] KALMALI; `commandRpm=0`; hiçbir işitilebilir tork,
   vibrasyon veya motor hareketi OLMAMALI.
8. GÜVENLİ bir host-side yöntem mevcutsa (paylaşımlı ROS grafiğini
   kirletmeyen izole Python script) NaN ve ±Inf yayınla: beklenen
   `rejectCount` artar, `commandFinite = false`, diagnostics message
   `CMD_VEL_INVALID` (level WARN); motorlar hala hareketsiz [0,0,0,0] ve
   tüm B4 güvenlik invariantları korunur. Güvenli injector yoksa bu
   kriteri DEFERRED olarak işaretle ve yazılı not ekle; paylaşımlı ROS
   graph üzerinde asla NaN/Inf gönderme.
9. micro-ROS Agent'ı tamamen durdur (Ctrl+C kill the agent process)
   bekle ≥ 10 saniye, sonra aynı 115200 /dev/ttyACM0 parametreleriyle
   yeniden başlat. Pico'nun reset olmadan yeniden bağlandığını DOĞRULA:
   `/pico_b/heartbeat` 2 Hz olarak geri döner; `pico_b/cmd_vel`
   diagnostics snapshot INVALIDATED olur (CMD_VEL_NEVER_RECEIVED veya
   CMD_VEL_STALE / commandFresh=false) YENİDEN bir cmd_vel yayın gelinceye
   kadar; subscription otomatik olarak tekrar oluşturulur.
10. Agent STOPPED durumdayken (USB kablo çekilmiyorsa agent killed veya USB
    fiziksel olarak çıkarılırsa) Pico B YEREL görevleri kesintisiz çalışmaya
    DEVAM ETMELİ: SBUS failsafe → röle 1/2 HEMEN OFF + servo center
    1570 us + synthetic safe-neutral 1002; motor slot scheduler 20 Hz
    0 RPM + brake 0xFF devam; OLED sayfaları görünür ve CH3 manual paging
    SADECE gerçek sağlıklı neutral ile edge-triggered çalışır; battery ADC
    ölçümü devam eder. USB/Agent geri bağlandıktan sonra kriter 3-7
    tam olarak tekrar çalışmalı.
11. 5 dakikalık tam tee yakalama (agent + diagnostics), AGENT_HANDOFF.md
    NEXT EXACT COMMANDS listesindeki komutlarla:
    `motorCycleHz ≈ 20` (19.8–20.2); `motorTransactionHz ≈ 80` (79–81);
    `temperatureQueryHz ≈ 4` (3.8–4.2) — 4 motor × 1 Hz phased + pending
    carry mekanizması.
12. Tüm 5 dakikalık yakalama süresince `onlineMask = 0x0F`;
    `faultMask = 0x00`.
13. 5 dakikalık yakalamada HİÇBİR `MOTOR_OFFLINE` veya `MOTOR_FAULT`
    diagnostic mesajı GÖRÜLMEMELİ (kriter 14 kablosu çekme senaryosu
    haricinde — o senaryoda sadece ilgili motor `OFF` olmalı, diğerleri
    `OK` kalmalı, global reset YOK).
14. Röleler + servo B4 regression: boot anında GP12/GP13 HIGH/OFF;
    SBUS CH7/CH8 active-LOW ≥ 1400 ON / ≤ 600 OFF; SBUS transmitter
    KAPANDIĞINDA iki röle HEMEN OFF ve `servoFailsafeCount` +
    `relayFailsafeCount` ARTAR; transmitter yeniden bağlanınca gerçek
    OFF→ON re-arm path ile çalışır (stuck ON olmaz). RDS3235 CH4 → GP6
    mapping 550/1570/2450 us kalibrasyonu korunur ve 90 deg/s slew
    gözlenebilir; input unhealthy → center 1570 us hedef slew.

On completion APPEND ONLY a new TEST_RESULTS.md entry #13 with explicit
PASS/FAIL/DEFERRED per 14 criterion; DO NOT rewrite entries 1-12. Hardware
PASS claim is NEVER allowed unless all 14 criteria pass.

## LESSIEN wheel-command receive-only build (2026-10-01)

This is a compile/static-only receive path. It does not command motors.

Topic: `/pico_b/wheel_commands`, message:
`lessien_interfaces/msg/WheelCommand`. Fixed velocity order is
`[FL, RL, FR, RR]` in rad/s. The canonical interface is
`/home/yusuf/12_task/lessien_interfaces/msg/WheelCommand.msg`, SHA256
`cb0405a45866b3cc63c502e0e726fe0fb00bdeaa425f7eb4874accd595472d6b`.
The Pi definition must match this hash exactly.

The final build enables
`PICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER=1`, disables
`PICO_B_ENABLE_CMD_VEL_SUBSCRIBER=0`, and uses only the separate Pico B
library. Both subscribers enabled together are rejected at compile time.
Motor target remains compile-time 0 RPM and brake remains 0xFF.

```bash
arduino-cli compile --clean --warnings all \
  --fqbn rp2040:rp2040:rpipico2 \
  --library /home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy_pico_b \
  --build-path /tmp/pico_b_wheel_command_rx_only_final \
  --build-property 'compiler.cpp.extra_flags=-DPICO_B_MOTOR_CYCLE_HZ=20 -DPICO_B_ENABLE_OLED=1 -DPICO_B_ENABLE_MANUAL_OLED_PAGING=1 -DPICO_B_ENABLE_RELAYS=1 -DPICO_B_ENABLE_CAMERA_SERVO=1 -DPICO_B_ENABLE_CMD_VEL_SUBSCRIBER=0 -DPICO_B_ENABLE_WHEEL_COMMAND_SUBSCRIBER=1 -DPICO_B_SOURCE_TIME_MAPPING_VERIFIED=0' \
  /home/yusuf/12_task/PICO_B_MICROROS_CONTROL_NODE
```

Freshness is intentionally split. Local receive age is wrap-safe and invalid at
250 ms. `sample_time_unix_us` is not trusted merely because the micro-ROS epoch
sync API returned success. The delivered build has
`PICO_B_SOURCE_TIME_MAPPING_VERIFIED=0`, so diagnostics can show a locally fresh
snapshot while sourceTrusted/sourceFresh/combined commandFresh remain false.
Clock error/drift must be measured and explicitly verified before any motion
stage.

Final UF2 SHA256:
`cfc672b3f8e529df58be2c62dfeb581291d67c6efd6d956c5a91bb5efc942be8`.
No upload or hardware test was performed.
