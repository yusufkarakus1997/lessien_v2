## Latest successful read-only motor capture — 2026-09-30 23:37:13 +03:00

This result supersedes the earlier no-data connectivity observation. Historical captures are retained.

Domain 0; capture 60.040466 s; 1201 RPM messages, 120 diagnostics messages, 0 rosout messages. RPM receive rate: 19.999962 Hz. RPM and diagnostics each had one publisher before capture and at the end. Agent was not stopped or changed.

Diagnostics rotated through 24 status groups: five samples per group, approximately 12 seconds apart. Values below are observed samples, not continuous guarantees. Diagnostics timestamps were zero; receive timestamps are stored in JSONL.

Commanded RPM: diagnostics commandRpm=0 in all five samples (6.769–54.770 s). /cmd_vel receiveCount=0 and CMD_VEL_NEVER_RECEIVED was reported. This is a firmware-reported command value, not independent observation of motor-bus commands.

onlineMask=0x0F and faultMask=0x00 in all five motor-summary samples. Each motor reported online=true, errorHex=0x00 and status OK in its five status samples.

| Motor | Measured topic RPM counts | Diagnostics rpm | currentA observed range | temperatureC | errorHex |
|---|---|---|---|---|---|
| M1 | 0: 1187; -1: 14 | 0 | -0.97 to -0.95 | 43 | 0x00 |
| M2 | 0: 1181; -1: 20 | 0 | -0.04 to -0.01 | 32 | 0x00 |
| M3 | 0: 1200; -1: 1 | 0 | -0.50 to -0.48 | 35 | 0x00 |
| M4 | 0: 1200; -1: 1 | 0 | -0.71 to -0.66 | 45 | 0x00 |

Current values are signed values exactly as reported; no calibration or physical interpretation is inferred. Sparse diagnostics rpm samples were all zero, while the faster RPM topic included -1. Their sampling times differ. -1 is neither classified as normal nor proof of a fault.

### CRC / timeout counter differences

Differences are last received minus first received for each field, not exact 60-second boundary snapshots. Each field has five samples spanning approximately 48 seconds within the capture.

| Status | Counter | First time (s) | Last time (s) | Initial | Final | Delta |
|---|---|---|---|---|---|---|
| pico_b/motor_comm_12 | M1_crc | 9.267 | 57.267 | 0 | 0 | 0 |
| pico_b/motor_comm_12 | M1_timeout | 9.267 | 57.267 | 0 | 0 | 0 |
| pico_b/motor_comm_12 | M2_crc | 9.267 | 57.267 | 0 | 0 | 0 |
| pico_b/motor_comm_12 | M2_timeout | 9.267 | 57.267 | 0 | 0 | 0 |
| pico_b/motor_comm_34 | M3_crc | 9.768 | 57.768 | 0 | 0 | 0 |
| pico_b/motor_comm_34 | M3_timeout | 9.768 | 57.768 | 0 | 0 | 0 |
| pico_b/motor_comm_34 | M4_crc | 9.768 | 57.768 | 0 | 0 | 0 |
| pico_b/motor_comm_34 | M4_timeout | 9.768 | 57.768 | 0 | 0 | 0 |
| pico_b/motor_detail_1_fl | speedTimeout | 10.267 | 58.267 | 0 | 0 | 0 |
| pico_b/motor_detail_1_fl | temperatureTimeout | 10.267 | 58.267 | 0 | 0 | 0 |
| pico_b/motor_detail_1_fl | speedCrcError | 10.267 | 58.267 | 0 | 0 | 0 |
| pico_b/motor_detail_1_fl | temperatureCrcError | 10.267 | 58.267 | 0 | 0 | 0 |
| pico_b/motor_detail_2_rl | speedTimeout | 10.770 | 58.770 | 0 | 0 | 0 |
| pico_b/motor_detail_2_rl | temperatureTimeout | 10.770 | 58.770 | 0 | 0 | 0 |
| pico_b/motor_detail_2_rl | speedCrcError | 10.770 | 58.770 | 0 | 0 | 0 |
| pico_b/motor_detail_2_rl | temperatureCrcError | 10.770 | 58.770 | 0 | 0 | 0 |
| pico_b/motor_detail_3_fr | speedTimeout | 11.267 | 59.267 | 0 | 0 | 0 |
| pico_b/motor_detail_3_fr | temperatureTimeout | 11.267 | 59.267 | 0 | 0 | 0 |
| pico_b/motor_detail_3_fr | speedCrcError | 11.267 | 59.267 | 0 | 0 | 0 |
| pico_b/motor_detail_3_fr | temperatureCrcError | 11.267 | 59.267 | 0 | 0 | 0 |
| pico_b/motor_detail_4_rr | speedTimeout | 11.770 | 59.769 | 0 | 0 | 0 |
| pico_b/motor_detail_4_rr | temperatureTimeout | 11.770 | 59.769 | 0 | 0 | 0 |
| pico_b/motor_detail_4_rr | speedCrcError | 11.770 | 59.769 | 0 | 0 | 0 |
| pico_b/motor_detail_4_rr | temperatureCrcError | 11.770 | 59.769 | 0 | 0 | 0 |

### Status messages and limits

No MOTOR_OFFLINE or MOTOR_FAULT messages occurred in captured diagnostics; rosout contained no messages. This does not establish absence of transient events between rotating diagnostics samples. MOTOR_COMM_WARNING occurred in all five motor-summary samples despite zero sampled CRC/timeout counters, onlineMask=0x0F and faultMask=0x00. Cause remains unestablished. lastOfflineMotorId stayed 4; this does not establish a new offline event during the capture.

Other observed statuses: BATTERY_CRITICAL (reported voltage 15.88–15.93 V), SBUS_UNHEALTHY, RELAY_INHIBIT, CMD_VEL_NEVER_RECEIVED. These are device-reported statuses; threshold correctness was not investigated. micro_ros agentState remained RUNNING in five samples, reconnectCount 1→1, and all three publish-error counters remained zero.

Raw artifacts: motor_checks/20260930_233713/pico_b_motors_rpm.jsonl, diagnostics.jsonl, rosout.jsonl, metadata.json. Sensor-data QoS; no packet-loss guarantee. No motor commands, firmware writes, daemon restart or Agent/container changes were performed.
