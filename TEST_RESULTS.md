# Pico B micro-ROS test results

Observed on 2026-09-30 (Europe/Istanbul), Ubuntu 24.04.4 ARM64, ROS 2 Jazzy.

| Check | Observed result |
|---|---|
| Docker service | active |
| Agent processes | running; live Pico messages received |
| Domain 42, no daemon | /parameter_events, /rosout only |
| Domain 0, no daemon | /cmd_vel, /diagnostics, /parameter_events, /pico_b/battery/voltage, /pico_b/heartbeat, /pico_b/motors/rpm, /pico_b/sbus/channels, /rosout |
| Heartbeat | PASS: 30 samples, 2.000321 Hz |
| motors/rpm rate | PASS: 301 samples, 20.002163 Hz |
| motors/rpm all zero | FAIL: unique values (0,0,0,0), (0,0,-1,0), (0,0,0,-1) |
| /cmd_vel subscriptions | PASS: 1, pico_b_control_node, BEST_EFFORT / VOLATILE |
| /cmd_vel publishers | 0 at inspection |
| Recent Agent logs/errors/disconnects | Unverified: Docker permission denied; sudo requires password |
| Container ROS/DDS environment | Unverified: same access limitation |
| Host network | --net=host observed in process invocation; Docker inspect unavailable |

Rates were calculated as (sample_count - 1) / (last_receive_time - first_receive_time), using monotonic receive timestamps in one 15-second rclpy sampling interval with sensor-data QoS. Heartbeat type: std_msgs/msg/UInt32; RPM type: std_msgs/msg/Int16MultiArray. No commands were published.

The domain correction was verified using ROS_DOMAIN_ID=0 in diagnostic subprocesses. Source pico_b_env.sh to apply it to another host terminal. Existing terminal environments were not modified.


## Historical no-data motor check — 2026-09-30 23:32:37 +03:00

At this historical capture, live connectivity was absent; see the latest successful capture below for current results.

Domain 0 was monitored for 60.019556 seconds (start: 2026-09-30T20:32:37.631510+00:00). Sensor-data QoS subscriptions received zero messages on /pico_b/motors/rpm, /diagnostics and /rosout. All three topics had zero discovered publishers at the end. JSONL files are intentionally empty because no messages arrived; they do not represent zero RPM or zero errors.

Docker service was active and the Pico by-id serial device existed. The process listing contained no micro_ros_agent or docker run process. No-daemon topic discovery in both domains 0 and 42 showed only /parameter_events and /rosout before capture. No Agent restart or container change was attempted in this read-only task. Container state was not established with Docker inspect.

| Requested field | Current result |
|---|---|
| commandRpm (commanded RPM) | Unavailable: no diagnostics received; not assumed zero |
| onlineMask / faultMask | Unavailable; not assumed zero |
| M1 rpm / currentA / temperatureC / errorHex | All unavailable |
| M2 rpm / currentA / temperatureC / errorHex | All unavailable |
| M3 rpm / currentA / temperatureC / errorHex | All unavailable |
| M4 rpm / currentA / temperatureC / errorHex | All unavailable |
| CRC initial / final / delta | Unavailable: no counter samples |
| Timeout initial / final / delta | Unavailable: no counter samples |
| MOTOR_OFFLINE / MOTOR_FAULT | No messages received; absence of faults cannot be established |

The previously observed M3/M4 -1 RPM readings remain unexplained. They are not classified as normal or faulty, and cannot be compared against commandRpm without contemporaneous diagnostics.

Artifacts: motor_checks/20260930_233237/metadata.json, pico_b_motors_rpm.jsonl, diagnostics.jsonl and rosout.jsonl (all three JSONL files in the same directory).

Next prerequisite: restore the Agent connection and verify active topic publishers, then repeat the 60-second read-only capture. No motor command was published; no firmware, package, daemon or container was changed.


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


## MOTOR_COMM_WARNING kaynak incelemesi — 2026-10-01

Sonuç: Pi üzerinde incelenen erişilebilir konumlarda Pico B firmware kaynağı bulunamadı; `lessien_ws/src` boş. Bu nedenle uyarıyı üreten koşul, bağlı olduğu ek sayaç/durum, eşik, sıfırlama davranışı ve kaynak satırları doğrulanamadı. Çalışan firmware sürümü tahmin edilmedi; tanılamadaki `bootStage=B3A` alanı sürüm/commit kanıtı kabul edilmedi.

Arama kapsamı: çalışma alanındaki `MOTOR_COMM_WARNING` eşleşmeleri yalnızca raporlarda ve kayıtlı tanılamalarda bulundu. `/home/pi5`, `/opt`, `/srv`, `/mnt`, `/media`, `/tmp`, `/usr/local/src` altında firmware/proje dosya adları arandı; `.codex` ve `.vscode-server` araç dizinleri dışlandı. Ek olarak `/home/pi5`, `/opt/ros`, `/usr/local/src`, `/srv`, `/mnt`, `/media` altındaki C/C++/Arduino/Rust/Python kaynaklarında `MOTOR_COMM_WARNING`, `tempGuardRejectCount`, `motorSlotMissCount` arandı (araç/önbellek dizinleri dışlandı); eşleşme yok. `/opt/containerd` ve bazı root sahipli `/tmp` dizinleri erişim engeli nedeniyle incelenemedi. Dolayısıyla tüm Pi dosya sisteminde kesin yokluk iddia edilmiyor; konteyner içeriği doğrulanmadı.

### Kayıt kanıtı; firmware kaynak satırları değildir

`motor_checks/20260930_233713/diagnostics.jsonl`:

- Satır 14, 38, 62, 86, 110: `MOTOR_COMM_WARNING`, `onlineMask=0x0F`, `faultMask=0x00`, `commandRpm=0`; `motorCycleHz=19.7–20.1`.
- Satır 19–24 ve her 24 satırdaki tekrarları (son grup 115–120): dört motorun CRC, timeout, wrongId ve speed/temperature CRC/timeout sayaçları ile `lateResponseCount` tüm beş örnekte 0.
- Satır 3, 27, 51, 75, 99: `tempScheduleMissCount=1633→1647`.
- Satır 4, 28, 52, 76, 100: `motorSlotMissCount=936→936`, `motorSlotLatenessMaxUs=1032387→1032387`.
- Satır 5, 29, 53, 77, 101: `tempDeferredCount=1633→1648`, `tempGuardRejectCount=16255→16401`, `tempStartLatenessMaxUs=1543→1543`, `speedConsecutiveFailMax=1→1`, `lastOfflineMotorId=4→4`.

Ek mevcut kayıt `/tmp/pico_b_after_battery_change.txt`: uyarı satır 57, 633, 1209'da; `tempScheduleMissCount=178` satır 382–383 ve 958–959'da; `motorSlotMissCount=2720` satır 404–405 ve 980–981'de; `tempDeferredCount=178`, `tempGuardRejectCount=23199`, `speedConsecutiveFailMax=1`, `lastOfflineMotorId=0` satır 422–431 ve 998–1007'de. Bu kayıt ayrı değerlendirildi; önceki JSONL ile aynı oturum olduğu varsayılmadı ve yeni canlı ölçüm yapılmadı.

Bu sıfır olmayan alanlar yalnızca gözlemdir: hiçbirinin warning koşuluna bağlı olduğu kaynakla kanıtlanamadı. Özellikle `lastOfflineMotorId=4` zorunlu koşul olarak gösterilemez; ek kayıtta bu alan 0 iken de uyarı görülüyor. Gruplar farklı zamanlarda yayınlandığından eşzamanlı durum garantisi yoktur. Kaynak bulunmadan kalıcı sayaç, anlık eşik veya latched uyarı yorumu yapılamaz.

Devam için gereken: uyarıyı yayınlayan firmware kaynağı ve çalışan imajla ilişkisini doğrulayan build/commit bilgisi. Ardından mesajın koşulu, kullandığı sayaçların artırıldığı/sıfırlandığı yerler ve koşul önceliği dosya/satır ile izlenmeli.

Yalnızca `AGENT_HANDOFF.md` ve `TEST_RESULTS.md` güncellendi. Kod değişikliği, firmware yükleme, motor komutu, servis/Agent/konteyner değişikliği yapılmadı. Doğrulama mevcut kayıtların okunması ve kaynak aramasıyla sınırlı; firmware koşulu testi kaynak eksikliği nedeniyle gerçekleştirilemedi.


## Firmware kaynağıyla doğrulanan warning koşulu — 2026-10-01

Bu inceleme, önceki bölümün kaynak bulunamadığı sonucunu günceller: kullanıcı tarafından belirtilen kaynak artık mevcut ve okundu. Önceki inceleme ve ölçümler tarihsel kayıt olarak korunmuştur.

Kaynak: `/home/pi5/lessien_ws/PICO_B_MICROROS_CONTROL_NODE/pico_b_node_app.cpp`

SHA256: `7c486db9afec214260a7745f7f5e51b28ba916d36da63c9ade4b7faab670559c`

**Kaynak ile çalışan firmware sürümünün/imajının eşleşmesi doğrulanmamıştır.** SHA256 yalnızca incelenen kaynak dosyasını tanımlar; yüklü imajı veya build seçeneklerini kanıtlamaz. Çalışan sürüm tahmin edilmedi. Aşağıdaki kaynak referanslarının tamamı bu dosyaya aittir.

### Kesin kaynak koşulu

`motorCommunicationWarningPresent()` (2489–2499): herhangi bir motor için aşağıdaki ifade doğruysa true döner:

```cpp
status.online &&
(status.crcErrorCount != 0 || status.timeoutCount != 0 ||
 status.wrongIdCount != 0 || status.lengthErrorCount != 0)
```

`fillMotorsDiagnostic()` (2501–2516) öncelik sırası: `faultMask != 0` ise MOTOR_FAULT; aksi halde `onlineMask != 0x0F` ise MOTOR_OFFLINE; aksi halde yukarıdaki fonksiyon true ise WARN seviyesinde MOTOR_COMM_WARNING (2513–2515). Dolayısıyla uyarının yayınlanması tüm motorların taze/online ve fault maskesinin sıfır olmasına ek olarak en az bir motorda bu dört hata sayacından birinin sıfırdan farklı olmasını gerektirir. Online maskesi 2469–2476'da `motorTelemetryFresh()` ile hesaplanır; bu fonksiyon online ve son geçerli hız yanıtı yaşının en fazla 250 ms olmasını kontrol eder (1288–1292; sabit 145). `commandRpm`, `motorCycleHz`, `lateResponseCount`, `speedConsecutiveFailMax` ve `lastOfflineMotorId` warning ifadesinde kullanılmaz.

### İstenen dört sayacın kontrolü

| Tanılama alanı | Kaynak değişkeni | Warning koşulunda? | Kanıt satırları |
|---|---|---|---|
| motorSlotMissCount | motorSlotMissCount | Hayır | Artış 1841–1844; yayın 2662 |
| tempScheduleMissCount | motorTemperatureScheduleMissCount | Hayır | Artış 1629–1636; yayın 2648–2649 |
| tempDeferredCount | motorTempDeferredCount | Hayır | Artış 1629–1636; yayın 2674 |
| tempGuardRejectCount | motorTempGuardRejectCount | Hayır | Artış 1644–1665; yayın 2675 |

Bu dört sayacın değerleri 2489–2516'daki karar ağacına girdi değildir. Önceki raporda sıfır olmayan gözlemler olarak listelenmeleri nedensellik kanıtı değildir; bu kaynak için doğrudan warning tetikleyicileri olmadıkları artık doğrulanmıştır.

### Birikimli geçmiş ve yayınlanmayan hata sayacı

Dört iletişim hata sayacı `uint32_t` alanlardır (376–379); `motorStatus` global sıfır başlangıcıyla oluşturulur (501). `recordMotorCommunicationError()` timeout/CRC/length/wrong-ID durumlarında ilgili sayacı artırır (1422–1449). Dosyadaki tüm kullanımlar tarandı: bu dört sayaç için başarı sonrası, periyodik pencere veya tanılama yayını sırasında temizleme/azaltma yok. `initializeMotorTask()` da bunları açıkça sıfırlamaz (1693–1735). Başarılı hız yanıtı yalnızca ardışık hata sayacını sıfırlar, son geçerli zamanı günceller ve motoru online yapar (1402–1419).

Sonuç: warning, güncel hata oranına/deltasına değil, birikimli hata geçmişine dayanır; güncel online/fault durumu mesaj önceliğini belirler. Tek bir geçmiş hata, sonraki iletişim başarılı olsa da sayaç sıfırdan farklı kaldığı sürece warning üretmeye yeterlidir. Ayrı bir warning latch değişkeni yoktur; koşul her dolumda tekrar hesaplanır. uint32 taşması dışında normal çalışma içinde bu sayaçları temizleyen bir yol bu dosyada görülmedi.

Gözden kaçan ek alan **lengthErrorCount**: `validateMotorResponse()` içinde timeout olmadığı halde `motorRxByteCount` veya `motorRxStored` 10 değilse LENGTH_ERROR döner (1470–1476; DDSM115_FRAME_SIZE=10 satır 128). Bu kontrol CRC ve ID kontrolünden önce yapılır (1478–1483); hata kaydında yalnızca lengthErrorCount artar (1444–1446), CRC/timeout sayaçlarının artması gerekmez. Hem hız hem sıcaklık yanıt yolları hata kaydedebilir (1513–1555).

`lengthErrorCount` tanılamaya yayınlanmıyor: motor_comm sayfaları yalnızca toplam CRC/timeout/wrongId yayımlar (2556–2576); motor_detail sayfaları speed/temperature CRC/timeout, son rxLengthLast ve lateResponseCount yayımlar (2579–2599). `rxLengthLast=10`, geçmiş lengthErrorCount değerinin sıfır olduğunu kanıtlamaz. motor_comm ve motor_detail sayfalarının OK durumu da sabit atanır (2562–2563, 2589–2590), bu sayaçların tümüyle sağlıklı olduğunu göstermez.

### Batarya değişimi sonrası kayıt — eski motor_checks ile birleştirilmedi

Bu bölümde yalnızca `/tmp/pico_b_after_battery_change.txt` incelendi. Eski `motor_checks/20260930_233713` ve `20260930_233237` kayıtları son batarya değişimi sonrası ölçüm olarak kullanılmadı; oturumlar arasında sayaç farkı hesaplanmadı.

- MOTOR_COMM_WARNING: satır 57, 633, 1209. İlgili özetlerde motorCycleHz=20.0, onlineMask=0x0F, faultMask=0x00, commandRpm=0 (62–69, 638–645, 1214–1221).
- Dört motorun toplam CRC/timeout/wrongId değerleri 0 (180–215, 756–791, 1332–1367). Speed/temperature CRC/timeout ve lateResponseCount değerleri de mevcut iki detay turunda 0 (228–311, 804–887).
- lengthErrorCount bu kayıtta yok; değeri, hangi motorda arttığı ve ne zaman oluştuğu ölçümden belirlenemiyor.

**Koşullu açıklama:** çalışan firmware bu kaynakla aynıysa ve sayaç örnekleri warning anındaki değerleri temsil ediyorsa, CRC/timeout/wrongId sıfırken uyarıyı açıklayan kalan koşul en az bir online motorun lengthErrorCount değerinin sıfırdan farklı olmasıdır. Bu, kaynak mantığından çıkan açıklamadır; çalışan cihazdan okunmuş bir lengthErrorCount ölçümü veya doğrulanmış kök neden değildir. Dönen tanılama sayfaları eşzamanlı değildir; kaynak–imaj eşleşmesi de doğrulanmamıştır.

Doğrulama: SHA256 inceleme başında ve rapor güncellemesi sırasında aynı; kaynak koşulu, tüm hata sayacı referansları, tanılama eşlemeleri ve batarya sonrası kayıt alanları salt-okunur incelendi. Yalnızca AGENT_HANDOFF.md ve TEST_RESULTS.md güncellendi. Kod değişikliği, build, firmware yükleme, yeni canlı ölçüm, motor komutu veya servis/konteyner değişikliği yapılmadı.


## LESSIEN URDF/Xacro ve ros2_control mock — 2026-10-01

**MOCK PASS; hardware PASS değildir.** Bu yeni görev eski Pico motor ölçümlerinden ve batarya değişimi sonrası kayıttan bağımsızdır. Eski rapor içeriği ve motor_checks kayıtları korundu. Çalışma kökü/üst dizinlerinde veya yeni paket ağacında uygulanabilir AGENTS.md bulunmadı; mevcut AGENT_HANDOFF.md ve TEST_RESULTS.md okundu.

### Teslim edilen dosyalar

- `src/lessien_description/urdf/lessien.urdf.xacro`: base_footprint, base_link, dört wheel link ve continuous joint; box/cylinder visual/collision, parametrik ataletler. x ileri, y sol, z yukarı; tüm wheel joint eksenleri +Y. ID1 FL, ID2 RL, ID3 FR, ID4 RR belgelemesi.
- `src/lessien_description/config/geometry.yaml`: paylaşılan launch varsayılanları; `launch/description.launch.py`: ayrı robot_state_publisher.
- `src/lessien_bringup/config/controllers.yaml` ve `launch/mock.launch.py`: yalnız mock_components/GenericSystem; joint_state_broadcaster + diff_drive_controller. Velocity command/state, position_feedback=false, open_loop=false. Görsel wheel TF için GenericSystem calculate_dynamics=true ve position state entegrasyonu.
- `src/lessien_bringup/scripts/verify_mock.py`: GenericSystem/namespace/type kontrolünden sonra sadece mock topicine zaman damgalı TwistStamped yayınlayan headless entegrasyon testi.
- Kök `README.md`: varsayımlar, eksik ölçüler, resmi kaynaklar, build/launch/test ve izolasyon komutları.

Doğrulanmış wheel_radius=0.05035 m, gövde=0.32×0.195×0.10 m. **UNVERIFIED, yalnız mock:** toplam sağ-sol merkez mesafesi wheel_separation=0.24 m; toplam ön-arka merkez mesafesi wheelbase=0.22 m; wheel_width=0.025 m; gövde altı açıklığı=0.035 m; body_mass=2.0 kg; wheel_mass=0.15 kg/tekerlek. 10 cm ve 7.5 cm bildirimleri ikiyle çarpılmadı veya kesin aks ölçüsü kabul edilmedi. Geçici değerler bu bildirimlerin yorumu değildir. Launch, wheel_separation ve wheel_radius değerlerini aynı kaynaktan Xacro'ya ve controller'a geçirir. Gerçek donanım için bu varsayımlar kullanılmamalı; ölçülerin anlamı ve gerçek değerleri doğrulanmalı.

Diferansiyel kinematik Pi 5'te diff_drive_controller içindedir. RDS3235 kamera servosudur; direksiyon modeli, donanım sürücüsü, serial erişimi veya Pico komut köprüsü eklenmedi.

### Kurulu sürüm / topic / parametre doğrulaması

Yerel package.xml: diff_drive_controller=4.42.1, joint_state_broadcaster=4.42.1, controller_manager=4.48.0, hardware_interface=4.48.0, robot_state_publisher=3.3.4, xacro=2.1.1 (`mock_checks/versions.json`).

[Resmi Jazzy controller dokümanı](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html), [resmi 4.42.1 kaynak kodu](https://github.com/ros-controls/ros2_controllers/blob/4.42.1/diff_drive_controller/src/diff_drive_controller.cpp), kurulu C++ başlıkları ve runtime subscriber ile `~/cmd_vel` tipinin **geometry_msgs/msg/TwistStamped** olduğu doğrulandı. position_feedback=false velocity state seçer; wheel name/radius/separation, timeout, odometry ve TF-prefix parametreleri doğrulandı. Eski sürüme ait use_stamped_vel parametresi varsayılmadı. [GenericSystem resmi dokümanı](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html) ile calculate_dynamics entegrasyonu doğrulandı.

Test ortamı ROS_DOMAIN_ID=77, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST. Komut hedefi sabit `/lessien_mock/diff_drive_controller/cmd_vel`; `/cmd_vel` veya başka gerçek motor komut topicine yayın yapılmadı. TF, TF-static, joint_states, odometry ve robot_description `/lessien_mock` altında; frame isimleri `lessien_mock/` önekli. Controller manager'ın `/diagnostics`, ROS `/rosout` ve `/parameter_events` topicleri test domaininde global kalır; bunlar motor komut kanalı değildir. Launch namespace'i sabittir, domain izolasyonu belgelenen terminal ortamıyla sağlanır.

### Gerçekleştirilen kontroller

| Kontrol | Sonuç / kanıt |
|---|---|
| colcon build, iki paket | PASS; `mock_checks/build.log` |
| Xacro + check_urdf | PASS; 6 link, 5 joint, base_footprint kökü; `lessien_mock.urdf`, `check_urdf.log` |
| Parametrik geometri | PASS; varsayılan ve alternatif UNVERIFIED 0.26 m / 0.23 m akslarla, mock tag açık/kapalı; `model_validation.json` |
| Ayrı description launch | PASS; robot_description, sabit base TF, ros2_control tag yok; `description_validation.json` |
| Mock launch / plugin | PASS; GenericSystem active; `launch.log` |
| İki controller | PASS; joint_state_broadcaster ve diff_drive_controller active |
| Dört wheel arayüzü | PASS; dört velocity command available+claimed, dört velocity ve dört position state available |
| Controller ek arayüzleri | diff_drive_controller/linear/velocity ve angular/velocity chainable referansları da listelenir; fiziksel motor arayüzü değildir |
| İleri komut, 0.1 m/s | PASS; dört wheel state +1.9860973 rad/s; odometri linear.x=0.1, angular.z=0 |
| Yerinde sol dönüş, 0.5 rad/s | PASS; sol −1.1916584, sağ +1.1916584 rad/s; odometri linear.x=0, angular.z=0.5 |
| Komut timeout | PASS; ayar 0.5 s; son yayından sonra ilk sıfır hız gözlemi 0.539990 s; durdurma komutu gönderilmeden doğrulandı |
| joint_states / TF | PASS; dört joint, position entegrasyonu, altı TF kenarı, wheel xyz konumları, zemin teması ve +Y quaternion dönüşünün joint position ile uyumu |
| Alternatif akslarla runtime | PASS; wheel_separation=0.26 ve wheelbase=0.23 model/controller uyumu ve hareket/timeout testleri; `alternate_runtime.json` |
| Gerçek /cmd_vel | Test domaininde publisher yok; test betiğinin tek komut publisher'ı sabit mock topicinde |
| RViz görsel inceleme | YAPILMADI; SSH oturumunda DISPLAY yok; headless kontroller tamamlandı |
| Donanım hareket/kalibrasyon | YAPILMADI; hardware PASS iddiası yok |

Varsayılan runtime kanıtı: `mock_checks/runtime.json`, `verification.log`, `topics.txt`. Alternatif ölçüler ayrı `alternate_runtime.json`/`alternate_launch.log`/`alternate_verification.log` içinde; varsayılan sonuçlarla birleştirilmedi. Timeout ölçümü kullanıcı uzayında mesaj alım zamanına dayanır, gerçek zaman garantisi değildir. Launch logunda FIFO RT scheduling için Operation not permitted uyarısı görüldü; OS yetkileri değiştirilmedi. Komut yokken controller'ın timeout/braking uyarıları beklenen davranış. Mock ideal state takibi ve kinematik testidir; yük, kayma, motor yönü/polaritesi, seri iletişim veya fiziksel güvenlik doğrulaması değildir.

### Korunan durum ve devam

Test için başlatılan mock ve description süreçleri test sonunda durduruldu. Micro-ROS Agent durdurulmadı/değiştirilmedi; görev başında görülen Agent PID 2075 sonunda da mevcut. Firmware inceleme klasöründeki dört dosyanın SHA256 değerleri aynı (`mock_checks/preservation_check.json`). pico_b_node_app.cpp SHA256: `7c486db9afec214260a7745f7f5e51b28ba916d36da63c9ade4b7faab670559c`. Kaynak–çalışan firmware eşleşmesi önceki rapordaki gibi doğrulanmış değildir; bu görev eşleşme iddiası eklemez.

Firmware build/yükleme, paket kurulumu, Agent/konteyner yönetimi, commit/push/upload yapılmadı. Yalnız yeni ROS paketleri build edildi. Kullanıcıdan netleştirilecekler: 10 cm/7.5 cm ölçülerinin toplam mı yarı mesafe mi olduğu, gerçek aks merkezleri, wheel width, gövde açıklığı, kütle ve kütle merkezi. Çalıştırma için kök README'deki domain 77 komutları kullanılmalı; mock PASS donanıma geçiş onayı değildir.


## RViz görüntüleme desteği — 2026-10-01

**VISUAL NOT VERIFIED.** Kullanıcı laptopta görüntüyü kontrol edene kadar bu durum korunmalıdır. GUI başlatılmadı; SSH ortamında DISPLAY/WAYLAND_DISPLAY yok. Laptop erişimi ve iki cihaz arasındaki DDS veri akışı bu görevde test edilmedi. Headless yapılandırma PASS, görsel PASS veya hardware PASS değildir.

Eklenen `src/lessien_description/rviz/lessien_mock.rviz`: RobotModel, TF, Grid (0.1 m hücre). Fixed Frame=lessien_mock/odom; RobotModel TF Prefix=lessien_mock; Description Topic=/lessien_mock/robot_description, Reliable/Transient Local. `src/lessien_description/launch/rviz.launch.py` yalnız RViz başlatır ve /tf ile /tf_static topiclerini /lessien_mock altına remap eder. Controller, model publisher veya motor komut köprüsü başlatmaz; config'te hedef/pose/point yayınlama araçları yok. CMake install kuralına rviz dizini, package.xml'e rviz2 ve rviz_default_plugins bağımlılıkları eklendi. Mevcut lessien_bringup mock launch zaten gerekli topicleri sağlıyor; bringup kaynakları değiştirilmedi.

Kök README'ye tam Pi mock ve laptop RViz akışı eklendi: iki tarafta ROS_DOMAIN_ID=77, RMW_IMPLEMENTATION=rmw_fastrtps_cpp, ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; eski ROS_LOCALHOST_ONLY/static-peer ayarlarını temizleme, gerektiğinde karşı IP ile static-peer kullanımı, keşif/topic kontrolleri, laptop bağımlılıkları, scp ve yerel colcon build komutları. Önceki LOCALHOST ayarları tek cihaz testi içindir; iki cihaz komutları ayrı bölümde SUBNET kullanır. Laptopta yalnız lessien_description aktarılır/build edilir; Pi'nin ARM64 build/install çıktıları kopyalanmaz. Mevcut model primitive box/cylinder olduğundan mesh/texture/package URI kaynağı gerekmiyor; gelecekte mesh eklenirse kaynak paket ve install/build gereksinimleri README'de açıklandı. Bu görevde laptopa gerçek aktarım veya paket kurulumu yapılmadı.

### Görüntülenecek varsayılan geometri

wheel_separation=0.24 m (toplam sağ-sol merkez mesafesi), wheelbase=0.22 m (toplam ön-arka merkez mesafesi): **UNVERIFIED**, yalnız mock. Kullanıcının 10 cm ve 7.5 cm bildirimleri yorumlanıp kesinleştirilmedi. x ileri, y sol, z yukarı. Gövde merkezi base_footprint'ten 0.085 m yukarıda; bu yükseklik 0.035 m UNVERIFIED gövde altı açıklığını içerir.

| Motor / tekerlek | x (m) | y (m) | z / base_link (m) | z / base_footprint (m) |
|---|---:|---:|---:|---:|
| ID1 FL | +0.11000 | +0.12000 | −0.03465 | +0.05035 |
| ID2 RL | −0.11000 | +0.12000 | −0.03465 | +0.05035 |
| ID3 FR | +0.11000 | −0.12000 | −0.03465 | +0.05035 |
| ID4 RR | −0.11000 | −0.12000 | −0.03465 | +0.05035 |

x/y her iki gövde frame'inde aynıdır. Tablo Xacro çıktısından doğrulandı; odom/world konumu değildir. Parametreler değişirse koordinatlar yeniden hesaplanmalı. Wheel width=0.025 m, body_mass=2 kg, wheel_mass=0.15 kg varsayımları da UNVERIFIED kalır. Görsel teyit gerçek ölçü doğrulaması değildir.

### Kontroller ve korunan durum

- Colcon build: iki paket PASS; `mock_checks/rviz_support/build.log`.
- Installed RViz config YAML, üç display plugin'inin kurulu manifestte varlığı, topic QoS/fixed-frame/TF-prefix, launch Python sözdizimi ve `ros2 launch ... --show-args`: PASS. Bu kontroller RViz rendering çalıştırması değildir.
- Xacro ile üretilen dört joint merkezinin sayısal kontrolü ve mesh kullanılmadığının kontrolü: PASS.
- Kanıt: `mock_checks/rviz_support/validation.json`, `launch_inspection.log`.
- Firmware klasörü, lessien_bringup dosyaları, Xacro ve geometry.yaml hashleri görev başıyla aynı; başlangıç listesi `protected_before.json` içinde. Firmware/gerçek motor komut yolu değiştirilmedi, hiçbir hareket komutu yayınlanmadı, Micro-ROS Agent'a müdahale edilmedi. Commit/push/upload yapılmadı.
- Önceki mock hareket testleri tarihsel PASS olarak korundu; yalnız görüntüleme dosyaları değiştiği için bu görevde tekrar motor/mock hareket testi yapılmadı. Eski motor_checks ve batarya sonrası kayıtlarla sonuçlar birleştirilmedi.

Kullanıcının yapacağı görsel kontrol: laptopta README komutlarıyla açılan RobotModel/TF durumları, gövde, dört tekerleğin tarafları ve zemine oturuşu, Grid ve TF eksenleri. Bu teyit gelene kadar **VISUAL NOT VERIFIED**.


## Kullanıcı teyidi: Pi–laptop RViz görüntüleme VERIFIED — 2026-10-01

**Pi–laptop RViz görüntüleme bağlantısı: VERIFIED (kullanıcı teyidi).** Kullanıcı, RViz ekran görüntüsüyle modeli gördüğünü doğruladığını bildirdi: **Global Status: Ok**, gövde ve tekerlekler görüntüleniyor. Kanıt kaynağı bu kullanıcı bildirimidir; bu güncellemede agent tarafından yeni bir ekran görüntüsü incelemesi veya bağlantı testi yapılmadı.

Bu teyit, önceki RViz bölümlerindeki **VISUAL NOT VERIFIED** durumunu modelin görüntülenmesi ve Pi–laptop görüntüleme bağlantısı kapsamında günceller. Önceki kayıtlar o inceleme anının tarihsel sonuçları olarak korunmuştur. Tüm geometrik ayrıntıların, tekerlek yönlerinin veya gerçek ölçü uyumunun görsel olarak doğrulandığı çıkarımı yapılmaz.

**Geometri hâlâ UNVERIFIED:** `wheel_separation=0.24 m` ve `wheelbase=0.22 m` geçici mock değerleridir. Görüntüleme teyidi bu değerleri kesinleştirmez. Diğer doğrulanmamış genişlik, yerden açıklık, kütle ve atalet varsayımları da doğrulanmış sayılmaz.

**Hardware PASS iddiası yoktur.** Gerçek donanım hareketi, motor yönü/kalibrasyonu veya modelin gerçek fiziksel ölçülere uyumu bu teyidin kapsamında değildir. Önceki mock PASS sonuçları donanım doğrulamasından ayrı kalır.

Bu görevde yalnız AGENT_HANDOFF.md ve TEST_RESULTS.md güncellendi. Geometri, firmware, ROS paketleri ve gerçek motor komut yolu değiştirilmedi; komut yayını, build, firmware yükleme veya Micro-ROS Agent müdahalesi yapılmadı.


## Güncel geometri — kullanıcı ölçüm tanımı doğrulandı, 2026-10-01

Bu bölüm önceki 0.24/0.22 m geçici geometri ve 0.025 m lastik genişliği kayıtlarını
geçersiz kılar; eski testler tarihsel olarak korunmuştur.

Tek varsayılan kaynağı `src/lessien_description/config/geometry.yaml`:
wheel_radius=0.05035, wheel_width=0.045, wheel_separation=0.145,
wheelbase=0.1757 m; gövde 0.32×0.195×0.10 m. Kullanıcının doğruladığı
kenar boşlukları: ön-arka 0.075 m + çap 0.1007 m; sağ-sol iç yüzler
0.10 m + genişlik 0.045 m. Xacro doğrudan aynı YAML'ı göreli yoldan yükler.
İki launch zaten bu YAML'ı okur; mock controller radius/separation parametreleri
aynı launch değerlerinden oluşturulur. Controller YAML'sine kopya sayılar eklenmedi.

Gövde merkezine göre joint x/y (m): FL (+0.08785,+0.0725),
RL (-0.08785,+0.0725), FR (+0.08785,-0.0725), RR (-0.08785,-0.0725).
Yerden açıklık 0.035 m hâlâ UNVERIFIED; değiştirilmedi. Bu varsayımla joint
z=-0.03465 m, gövde merkezi zeminden 0.085 m. Kütle/atalet varsayımları da doğrulanmadı.

**Geometrik çakışma mevcut:** katı gövde box y=[-0.0975,+0.0975] m;
sol lastik y=[0.050,0.095], sağ lastik y=[-0.095,-0.050] m. Tüm lastik
genişliği gövde izdüşümünde; gövde dış lastik yüzünden her yanda 0.0025 m taşar.
Lastik x aralıkları ±[0.0375,0.1382] m, gövde x=[-0.16,0.16] m içindedir.
Gövde z=[0.035,0.135], lastik z=[0,0.1007] m; z izdüşüm örtüşmesi 0.0657 m.
Silindir merkez x kesitinde de kesişim vardır: dört lastikte hacimsel çakışma.
Gerçek gövde oyukları/şekli ve açıklığı bilinmediğinden bu katı box modelinin
sınırlamasıdır; fiziksel robotta çakışma olduğu iddia edilmez. Ölçüler otomatik
ayarlanmadı. GenericSystem temas çözmez; mock PASS çakışmasızlık kanıtı değildir.

### Doğrulama

- İki paket colcon build PASS; `mock_checks/geometry_update/build.log`.
- Xacro + check_urdf PASS; 6 link / 5 joint; `lessien_mock.urdf`, `check_urdf.log`.
- Kaynak ve kurulu Xacro varsayılanları, dört joint, cylinder/box ölçüleri,
  iki launch varsayılanı, explicit override ve boşluk hesapları PASS;
  `validation.json` (aynı kanıt dizininde).
- Mock GenericSystem ve iki controller active; dört velocity command claimed,
  position/velocity state arayüzleri PASS. Runtime radius=0.05035,
  separation=0.145, position_feedback=false, open_loop=false.
- İleri 0.1 m/s: dört tekerlek +1.9860973 rad/s; sol dönüş 0.5 rad/s:
  sol -0.7199603 / sağ +0.7199603 rad/s; odometri, position entegrasyonu ve TF PASS.
- Timeout 0.5 s; son yayından sıfır hıza gözlenen süre 0.540177 s: PASS.
- İlk deneme topic hazırlığı için sabit 1 s beklemede FAIL oldu; komut publisher'ı
  oluşturulmadan durdu. `runtime_initial.json`, `verification_initial.log` korundu.
  `verify_mock.py` hazır topicleri en fazla 10 s bekleyecek şekilde düzeltildi;
  tekrar PASS (`runtime.json`, `verification.log`). Motor komut hedefi değişmedi.
- Test ortamı domain 77 / LOCALHOST; yalnız `/lessien_mock/diff_drive_controller/cmd_vel`
  yayınlandı. Test domaininde `/cmd_vel` publisher sayısı 0. Gerçek motor testi yok.
- FIFO RT scheduling izni uyarısı ve SIGINT kapanışında pal_statistics invalid-context
  hata logları görüldü; launch iki node için clean exit bildirdi. OS ayarı değiştirilmedi.
- Firmware ve pico_b_env.sh SHA256 aynı: `protected_before.json`, `validation.json`.
  Firmware/gerçek motor komut yolu ve Micro-ROS Agent değiştirilmedi.
  Test için açılan mock süreçleri kapatıldı. Hardware PASS iddiası yok.

README yeni Pi launch ve laptop description kaynak aktarımı/build komutlarıyla
güncellendi. Pi'de domain 77 / SUBNET ortamında `ros2 launch lessien_bringup mock.launch.py`
yeni YAML varsayılanlarını kullanır. Laptopta yalnız `src/lessien_description`
scp ile yeniden aktarılıp yerel colcon build/source yapılmalı; ARM64 build/install
kopyalanmamalı. Bu görevde laptopa aktarım ve yeni geometriyle RViz görsel kontrolü
yapılmadı; önceki Pi–laptop görüntüleme kullanıcı teyidi tarihsel olarak geçerlidir.


## Kullanıcı teyidi: güncellenen genel yerleşim görsel olarak VERIFIED — 2026-10-01

Kullanıcı, güncellenen RViz modelinin gerçek robotun genel gövde/tekerlek
yerleşimine benzediğini doğruladı. **Genel yerleşim görsel olarak VERIFIED
(kullanıcı teyidi).** Kanıt bu kullanıcı bildirimidir; agent tarafından yeni
bir ekran görüntüsü incelemesi veya fiziksel ölçüm yapılmadı. Bu kayıt önceki
bölümdeki yeni geometrinin görsel kontrolünün beklediği durumunu bu kapsamda
günceller; önceki test kayıtları tarihsel olarak korunur.

| Kullanıcı ölçüsü | Değer |
|---|---:|
| wheel_radius | 0.05035 m |
| wheel_width | 0.045 m |
| wheel_separation | 0.145 m |
| wheelbase | 0.1757 m |
| Gövde uzunluk × genişlik × yükseklik | 0.32 × 0.195 × 0.10 m |

Yerden yükseklik/açıklık, kütle/atalet (inertia) ve basit box gövdenin
collision geometrisinin gerçek robotu temsil etmesi **UNVERIFIED** kalır.
Mevcut 0.035 m gövde altı açıklığı görsel teyitle doğrulanmış sayılmaz.
Önceki katı box–lastik çakışması bulgusu geçerlidir; genel görsel benzerlik
collision geometrisinin doğruluğunu veya çakışmasızlığını kanıtlamaz.

**Hardware sürüş PASS iddiası yoktur.** Bu teyit gerçek sürüş, motor yönü,
kalibrasyon veya fiziksel dinamik doğrulaması değildir. Önceki mock PASS
sonuçları kendi kapsamlarında korunur.

Bu görevde yalnız AGENT_HANDOFF.md ve TEST_RESULTS.md güncellendi ve eklenen
kayıtlar okunarak doğrulandı. Geometri/kod/firmware veya gerçek motor komut
yolu değiştirilmedi; build, yeni runtime testi, komut yayını veya Micro-ROS
Agent müdahalesi yapılmadı.


## Pi–Pico wheel komut arayüzü: receive-only uygulama planı — 2026-10-01

**Durum: PLAN; uygulanmadı, build/upload ve donanım sürüş testi yapılmadı.**
AGENT_HANDOFF.md, TEST_RESULTS.md, kök README ve Pico B firmware kopyası
incelendi. Aşağıdaki değerler/tasarım kararları öneridir; yeni runtime PASS değildir.

### Mevcut arayüz ve kaynak bütçesi

Kaynak: `PICO_B_MICROROS_CONTROL_NODE/pico_b_node_app.cpp`.

| Pico B topic | Mesaj | Kaynaktaki kullanım |
|---|---|---|
| /pico_b/heartbeat | std_msgs/msg/UInt32 | Publisher, 2 Hz |
| /pico_b/sbus/channels | std_msgs/msg/Int16MultiArray | Publisher, 16 değer, 20 Hz |
| /pico_b/battery/voltage | std_msgs/msg/Float32 | Publisher, geçerli ölçümde 2 Hz |
| /pico_b/motors/rpm | std_msgs/msg/Int16MultiArray | Publisher, 4 değer, ID1/2/3/4; yeni motor turunda |
| /diagnostics | diagnostic_msgs/msg/DiagnosticArray | Publisher, 2 Hz; tek status, en çok 6 KeyValue |
| /cmd_vel | geometry_msgs/msg/Twist | Opsiyonel receive-only subscriber; B5A flag=1 |

Beş uygulama publisher'ı; bir opsiyonel subscriber; service/client yok.
Publisher ve subscriber QoS: BEST_EFFORT / VOLATILE / KEEP_LAST(1).
Kaynak varsayılanı CMD_VEL flag=0 ve motor cycle=10 Hz; README B5A build'i
CMD_VEL=1 ve cycle=20 Hz seçiyor. Çalışan imajın flag'leri kaynak varsayılanından
çıkarılamaz. Motor RPM topic'i schema olarak değişken boyutlu olsa da kaynak
4 elemanlık statik buffer kullanıyor; bu, mesaj tipini sabit boyutlu yapmaz.

Kaynak kanıtları: topic/buffer sınırları 251–263; motor sırası/sıfır hedefler
430–460; subscriber callback 623–654; entity cleanup 703–737; motor hız TX ve
retry 1344–1375; entity creation/subscriber 1877–1956; RPM publisher 2199–2220;
diagnostic rotasyonu 2223–2358; executor spin 3150–3155; timeout 3220–3227.
24 diagnostic sayfası × 0.5 s yaklaşık 12 s/tur: tanılama yayını timeout saati
olarak kullanılamaz. Değer metin buffer'ı 24 byte; uzun liste tek alana sığdırılmaz.

Firmware README B5A bölümünün bildirdiği **ayrı Pico B** statik arşiv:
`/home/yusuf/12_task/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy_pico_b/`
PUB=10, SUB=1, SVC=0, CLNT=0, HIST=2, custom transport; Twist typesupport mevcut
olarak belgelenmiş. Bildirilen lib SHA256:
`e62ed07fd8accc84367c21ec8241caf6bcd05857afa9d1feba810327a95ce099`.
Bu bütçe **belge kanıtıdır**, Pi'de arşiv/config.h okunarak doğrulanamadı.
Pico A SUB=0 arşivine dokunulmayacak. README'nin baştaki “subscriber yok” ifadesi
eski B3 açıklamasıdır; B5A bölümü ve mevcut kaynak opsiyonel subscriber içerir.

Öneri: yeni WHEEL_COMMAND_RX_ONLY=1 build'inde CMD_VEL_SUBSCRIBER=0;
compile-time guard iki flag'in birlikte 1 olmasını reddetsin. Legacy /cmd_vel
kodu ayrı regression build'inde tutulur, yeni build'de /cmd_vel subscription
oluşmaz; Twist'ten Pico içinde wheel dönüşümü veya fallback yapılmaz. Böylece
PUB=5, SUB=1, executor handle=1, service/client=0 kalır. Yeni subscriber **tipi**
için aynı kapasitede fakat custom typesupport eklenmiş ayrı Pico B library
rebuild gerekir. İki subscriber birlikte istenirse ayrı B arşivinde SUB>=2,
executor=2 ve history/MTU/RAM yeniden ölçülmeden açılmaz; ilk aşama bunu içermez.

B arşivinin config.h, colcon.meta/CMake seçenekleri, generated message C/IDL,
libmicroros.a typesupport sembolleri ve gerçek link haritası kontrol edilmeli.
HIST=2, transport MTU ve XRCE stream history farklı kaynaklardır; tek sayıdan
paketin sığdığı sonucu çıkarılmaz. Beş publisher için 10 kapasite, custom tipin
mevcut olduğunu kanıtlamaz. İzole B build dizini/sketchbook ve yalnız B library
staging kopyası kullanılmalı; README'deki A klasörünü taşıma/library adı swap
adımları uygulanmamalı. Verbose build logu seçilen B yolu/hash'ini kanıtlamalı.

### Mimari, mesaj ve birimler

Pi diff_drive_controller -> dört ros2_control velocity command interface ->
Pi hardware plugin -> `/pico_b/wheel_commands` -> Pico bounded RX snapshot.
Pico kinematik hesaplamaz. Plugin joint adlarıyla eşler; arayüzlerin geliş sırasına
ve alfabetik sıralamaya güvenmez. Sıra sabit:
0=ID1 FL/front_left, 1=ID2 RL/rear_left, 2=ID3 FR/front_right, 3=ID4 RR/rear_right.

Önerilen `lessien_interfaces/msg/WheelCommand.msg` (yeni dosya henüz yok):

```text
uint64 source_session_id
uint64 sample_time_unix_us
uint32 sequence
uint32 protocol_version
float64[4] velocity_rad_s
```

protocol_version=1; session_id sıfır olamaz ve her Pi publisher aktivasyonunda
farklı üretilir. Dört değer joint koordinatında rad/s, +Y ekseninde pozitif ileri
yuvarlanmadır; motor elektriksel işaretleri wire mesajına uygulanmaz. String,
Header/frame_id veya MultiArray yok. Alan toplamı 56 byte; gerçek CDR uzunluğu,
encapsulation/padding/XRCE overhead ve RAM build sırasında ölçülür. Float64
ros2_control double komutlarını daraltmadan taşır; RP2350 üzerindeki maliyeti
ölçülür. 50 Hz'de yalnız alan yükü 2800 byte/s; bu transport kapasite PASS'i değildir.

Birim sözleşmesi (tek uygulama yeri gelecekte Pi plugin dönüşüm katmanı):
`joint_rpm = velocity_rad_s * 60/(2*pi)`;
`joint_rad_s = joint_rpm * 2*pi/60`.
1 rad/s = 9.549296586 RPM; 60 RPM = 2*pi rad/s.
Gelecekte motor protokol hedefi `raw_cmd_rpm[i] = command_sign[i] * joint_rpm[i]`;
feedback `joint_rad_s[i] = feedback_sign[i] * raw_rpm[i] * 2*pi/60`.
command_sign ve feedback_sign, her motor için ayrı doğrulanacak ±1 değerleridir;
şimdilik UNVERIFIED/unset, sağ/sol aynalama tahmini yok. Aynı oldukları da ölçümsüz
varsayılmaz. Formüller RPM alanının tekerlek çıkışını temsil etmesi içindir;
farklı şaft/aktarma oranı varsa ayrıca doğrulanmalıdır. Signed RPM decode mevcut
kaynakta 1402–1419'dadır, fiziksel yön kanıtı değildir.

Receive-only aşamada yalnız rad/s snapshot ve gerekirse açıkça “hesaplanan joint
RPM” tanılaması vardır; hiçbir dönüşüm motor TX'ine bağlı değildir. Gelecekte
int16 motor komutuna geçişte finite/range kontrolü, doğrulanmış hız sınırı ve
belgelenmiş yuvarlama gerekir; şu aşamada motor limiti veya işaret uydurulmaz.

### Pi gönderici ve Pico kabul/freshness sözleşmesi

- Pi plugin write() dört değeri tek snapshot olarak bounded RT buffer'a aktarır;
  DDS publish ve executor ayrı non-RT iş parçacığında, en yeni snapshot ile çalışır.
  Bir snapshot yeniden yayınlanırken yeni timestamp/sequence verilmez. Yeni
  control write örneği sequence artırır; ilk örnek seq=0 olabilir. Publisher
  nominal 50 Hz, QoS BEST_EFFORT/VOLATILE/KEEP_LAST(1); sınırsız kuyruk yok.
  Inactive/error/deactivate durumunda eski veriyi yeniden taze göstermeden yayın
  durur. Controller giriş timeout'u ve Pico transport timeout'u ayrı katmanlardır.
- Source timestamp snapshot örnekleme anının Pi system/Unix saatidir; ROS sim time
  kullanılmaz. Pi Agent aynı host saatini kullanmalı. Pico, Agent epoch sync ile
  saat eşlemesini doğrulamadan commandFresh=true üretmez. Yalnız yerel alım yaşı
  kuyrukta gecikmiş bir mesajı taze sanabileceğinden üretim yaşı da denetlenir.
- Önerilen test eşikleri: kabulde source age <=100 ms; izin verilen saat hatası
  <=20 ms; gelecek timestamp >20 ms reddedilir. Bunlar doğrulanmış donanım
  limitleri değil, ölçülmesi gereken RX test sözleşmesidir. Sync başarı bayrağı
  tek başına 20 ms doğruluk kanıtı değildir. Sync ölçümü/drift süresi testle
  sınırlandırılmalı; kalite yoksa TIME_UNSYNCED ve invalid kalır. Sync çağrısı
  callback'te veya motor deadline'ını bloke eden 1 s çağrı olarak yapılmaz;
  kısa bütçeli ayrı lifecycle adımı ve scheduler regresyon kapısı gerekir.
- Aynı source_session içinde unsigned `delta = sequence-last_sequence`;
  sadece `0 < delta < 2^31` yenidir. Duplicate/geri sıra/yarım aralık belirsizliği
  reddedilir; sequence gap kabul edilir ve sayılır. uint32 wrap testi zorunlu.
  Ayrıca sample timestamp geriye gitmez. İlk oturum ilk tamamen geçerli paketle
  bağlanır; RUNNING sırasında farklı session reddedilir. Pi publisher restart'ı
  yeni alıcı entity oturumu gerektirir; otomatik session takeover yok.
- Callback null/version/session/sequence/zaman ve **dört** değerin tamamında
  isfinite kontrolü yapar. Tek NaN/+Inf/-Inf tüm komutu reddeder. Kısmi wheel
  güncellemesi yok. Son kabul edilen dört değer tek işlemle kopyalanır; mevcut
  tek cooperative thread'de tutarlıdır. Core1 eklenirse bu varsayım geçersizdir.
  Callback allocation, publish, log, delay veya motor/relay/servo erişimi yapmaz.
- Reject sayacı/nedeni kaydedilir; reddedilen paket lastAcceptedRx/sequence'i
  ilerletmez, freshness süresini uzatmaz. Muhafazakâr politika: her reject aktif
  valid/fresh bayrağını düşürür; son iyi sayılar yalnız historical snapshot'tır.
  Sonraki tamamen geçerli yeni sequence yeniden fresh olabilir. Lifetime sayaçları
  session state'ten ayrıdır; receiveCount>0 hiçbir zaman “bu oturumda alındı” kanıtı olmaz.
- Timeout: yerel wrap-safe `uint32(now-lastAcceptedRxMs) >=250` ms veya source
  yaşının 250 ms'yi bulması invalid/stale yapar. Loop'ta uygulanır, 12 s diagnostic
  turu beklenmez. Invalid latched kalır; millis wrap eski snapshot'ı diriltemez.
  Zaman ileri/geri sıçraması veya sync kalitesinin kaybı invalidation üretir.
- Boot, bağlantı kaybı algısı, executor gerçek hatası, destroy, create başarısızlığı
  ve reconnect başlangıcında valid/fresh=false, aktif dört değer=0, has_sequence=false,
  has_current_session_sample=false, source_session binding temizlenir. Lifetime
  counters tutulur, eski snapshot varsa ayrı historical alanında etiketlenir.
  Entity creation + yeni time sync sonrası bir readiness timestamp sınırı alınır;
  ilk kabul edilen örnek readiness+20 ms'den sonra üretilmiş olmalıdır. Böylece
  önceki bağlantıdan bekleyen paket yeni oturumu tazelemez (saat hata sınırı şartıyla).
  Reconnect kendi başına snapshot'ı geçerli yapmaz; yeni post-ready paket şarttır.
  Aynı bağlantıda timeout sonrası da yeni kabul timestamp'i timeout sınırı+20 ms'yi
  aşmalıdır; sequence high-water mark korunur. Bu protokol kimlik doğrulama değildir;
  Pi'de tek yetkili publisher kontrolü ve beklenmeyen ikinci publisher testi gerekir.

Mevcut /cmd_vel invalidation sadece değerleri/iki bayrağı sıfırlar, lifetime
receiveCount/lastReceiveMs'yi temizlemez; yeni snapshot için bu desen körlemesine
kopyalanmayacak. Mevcut executorCreated flag'i add_subscription sonrası set olur;
executor init başarılı/add başarısız yolunda cleanup eksik kalabilir. Yeni lifecycle'da
support/node/subscriber/executor initialization durumları ayrı izlenip tüm kısmi
hatalarda idempotent cleanup test edilecek. spin_some dönüşü şu an atılıyor;
normal no-data/timeout ile gerçek hata ayrılmalı, gerçek hata snapshot'ı invalidate etmeli.

Motor invariants: `MOTOR_TARGET_RPM`, dört elemanlı constexpr hedef tablosu ve
static_assert'ler 0 kalır; speed frame bytes[2:4]=0, byte[7]=0xFF; retry aynı
sabit frame'i yollar. Callback/snapshot hiçbir TX setter'ına bağlanmaz. UART,
CRC, ID sırası, hız/sıcaklık scheduler, SBUS, relay ve servo davranışı korunur.

### Tanılama ve feedback sınırı

Mevcut /diagnostics publisher'ı içinde bounded sayfalar eklenir: dört kabul edilmiş
rad/s + sequence + freshness için bir sayfa; age/counters/reject reason/session
bilgileri için ayrı sayfalar. Her sayfa <=6 değer, 24-byte metin sınırı; 64-bit alan
formatı uint32 yardımcıya daraltılmadan yazılmalı. Snapshot sayfaları sequence ile
ilişkilendirilir; farklı rotasyon anları eşzamanlı sanılmaz. Timeout/invalidation
olayı sayacı ve son nedeni tutulur; kısa transient olayları sparse yayında kaybolmaz.
Yeni publisher/service gerekmez; sayfa sayısı artınca tam tur süresi yeniden raporlanır.

Mevcut RPM mesajında timestamp/sequence/online mask yok ve offline wheel 0 yayınlanıyor.
Dolayısıyla 0 RPM “geçerli hareketsiz feedback” olarak kabul edilemez. RX-only plugin
komut taşıma testleri harness/fake state ile izole yapılabilir; bu state gerçek
odometri sayılmaz. Fiziksel feedback'i ros2_control'a geçerli state olarak vermek,
kalibre edilmiş sign ve zaman/validity içeren feedback sözleşmesi tamamlanana kadar
ayrı kapıdır. Normal hardware activation bu eksikleri gizleyerek açılmamalı.

### Uygulamada değişecek dosyalar (bu görevde değiştirilmedi)

1. Yeni `src/lessien_interfaces/msg/WheelCommand.msg`, `CMakeLists.txt`, `package.xml`:
   sabit ROS/MCU type contract ve host typesupport.
2. Asıl kaynakla eşleştirildikten sonra `PICO_B_MICROROS_CONTROL_NODE/pico_b_node_app.cpp`:
   feature flag/guard, message buffer, RX snapshot/callback, lifecycle/time sync,
   bounded diagnostics. Motor TX işlevleri davranışsal olarak değiştirilmez.
3. Ayrı Pico B library build workspace'i: yeni interface paketi, gerçek colcon.meta,
   custom transport/RP2350 toolchain seçimi, generated headers ve libmicroros.a.
   Bu workspace Pi'de yok; tam değişiklik yolları asıl build ağacı alınınca belirlenir.
   Pico A arşivi/kaynak/library.properties hiçbir aşamada değiştirilmez.
4. Yeni `src/lessien_hardware/` plugin paketi: header/source, plugin XML, CMake,
   package.xml, bounded command publisher ve testleri. Gerçek joint telemetry validity
   tamamlanmadan üretim sürüş plugin'i olarak sunulmaz.
5. `src/lessien_description/urdf/` altında ayrı receive-only kontrol tanımı ve
   `src/lessien_bringup/launch/receive_only.launch.py` + ilgili config/test harness:
   açık RX-only seçim; mevcut mock.launch.py gerçek topic'e bağlanmaz.
6. Firmware README ve kök README: B-only build seçimi, contract, launch/test,
   eşleşmiş hash manifesti. AGENT_HANDOFF.md ve TEST_RESULTS.md yeni kanıtlarla güncellenir.

### Uygulama test kapıları

| Kapı | Geçme ölçütü | Bu görevde durum |
|---|---|---|
| G0 Provenance/toolchain | Asıl kaynak–Pi kopyası tüm dosya hash eşleşmesi; board/core/compiler/flags/B archive+config manifesti | AÇIK; asıl kaynak ve MCU araçları yok |
| G1 Type/library budget | Host+MCU aynı IDL; CDR roundtrip dört farklı değer; typesupport link; PUB5/SUB1/exec1; MTU/RAM/map ölçümü; A hash değişmez | YAPILMADI |
| G2 Saf fonksiyon testleri | ID/joint sırası, rad/s↔RPM, unset sign; her kanalda NaN/±Inf; duplicate/reorder/gap/wrap; null/version/session; 249/250/251 ms; millis wrap; source age/future/saat sıçraması | YAPILMADI |
| G3 Lifecycle/failure injection | Her create adımında hata; cleanup tekrarları; reconnect eski paket reddi; Pi restart; hiçbir eski snapshot fresh olmaz; 100 reconnect'te leak yok | YAPILMADI |
| G4 Compile/regression | RX-only=1/CMDVEL=0; legacy=1/wheel=0; ikisi=0 build; ikisi=1 compile fail; 0 RPM/0xFF assert; güçlü transport sembolleri; close Serial.end çağırmaz | YAPILMADI |
| G5 Pi zinciri, izole test | diff_drive çıktısı dört joint -> plugin -> test subscriber; 0.1 m/s için tümü 1.9860973 rad/s; 0.5 rad/s sol dönüş için sol -0.7199603, sağ +0.7199603; queue ve thread timeout | YAPILMADI |
| G6 Ayrı onaylı receive-only cihaz testi | Pozitif/negatif/sıfır ve invalid trafik altında snapshot doğru; speed TX/retry her ID'de bytes2/3=0 ve brake=FF; gerçek hareket yok; disconnect/reconnect/USB ve flooding regresyonu | YAPILMADI; upload bu görevin dışında |
| G7 Zamanlama/soak | 20 Hz build'de motorCycleHz 19.8–20.2, transaction 79–81, temp 3.8–4.2 Hz hedefleri; 5/30 dk capture, sayaç deltaları, OLED/SBUS/relay/servo regresyonu; sync ve RX flood deadline etkisi | YAPILMADI |

G6'da motor TX kanıtı için frame capture/logic analyzer veya doğrulanmış transport
instrumentation gerekir; yalnız diagnostics commandRpm=0 yeterli değildir.
Ölçülen RPM'deki önceki -1 örnekleri normal kabul edilmez ve komut sıfırından
çıkarımla silinmez; yeni kayıtta ayrı değerlendirilir. Test kapılarının tamamı
receive-only kapsamındadır, geçseler bile **hardware sürüş PASS değildir**.

### Pi'de gerçekten kontrol edilenler ve sınırlar

- Yerel cpp SHA256: `7c486db9afec214260a7745f7f5e51b28ba916d36da63c9ade4b7faab670559c`;
  önceki kayıtla aynı. Bu, asıl kaynak veya yüklü imajla eşleşme kanıtı değildir.
- /home altında yalnız pi5 görüldü; README'deki `/home/yusuf/12_task` yok.
  /home/pi5, /opt, /usr/local, /tmp erişilebilir dosyalarında (Codex/VS Code araç
  dizinleri dışlanarak) yapılan aramada ikinci cpp, libmicroros.a, colcon.meta,
  library.properties veya UF2 bulunmadı. Root erişim hataları dışlandı; tüm disk
  veya Docker container içeriğinde kesin yokluk iddiası değildir.
- PATH'te arduino-cli, arm-none-eabi-g++, ninja yok; ~/.arduino15, ~/Arduino,
  ~/.platformio yok. CMake, colcon, g++, make, cppcheck ve Docker CLI var.
  Jazzy rosidl_default_generators ve hardware_interface paketleri mevcut.
  Host paket önkoşulları var, **Pico firmware build hazır değil**; Docker CLI'nin
  varlığı doğru builder image/core/library'nin varlığı demek değildir.
- Domain 0 / SUBNET, no-daemon 3 s salt-okunur keşifte yalnız /parameter_events
  ve /rosout bulundu; /cmd_vel info “Unknown topic” döndü. Bu anda Pico bağlantısı
  doğrulanmadı; önceki başarılı ölçümler tarihsel kalır. Agent yeniden başlatılmadı.
- Bu görevde yalnız iki rapor dosyası değiştirildi. Kod/library/firmware değişikliği,
  build, kurulum, upload, motor komutu veya Agent/container yönetimi yapılmadı.
  Yeni interface/runtime testleri bu yüzden PASS olarak işaretlenmedi.

Teknik dayanaklar: sabit array ve IDL eşlemesi için
[ROS 2 IDL tanımı](https://design.ros2.org/articles/idl_interface_definition.html);
RMW statik kaynak ayarları için
[micro-ROS RMW kaynağı](https://github.com/micro-ROS/rmw_microxrcedds);
epoch sync API için
[micro-ROS resmi öğretici kaynağı](https://github.com/micro-ROS/micro-ros.github.io/blob/master/_docs/tutorials/programming_rcl_rclc/micro-ROS/micro-ROS.md).
Bu belgeler yerel B arşivinin sürüm/kapasite doğrulamasının yerine geçmez.


## WheelCommand receive-only test ortamı hazırlandı — 2026-10-01

Kullanıcının aktardığı lessien_interfaces build edildi; kaynak ve installed
WheelCommand.msg SHA256 beklenen değerle birebir aynı:
`cb0405a45866b3cc63c502e0e726fe0fb00bdeaa425f7eb4874accd595472d6b`.
Alan sırası: uint64 source_session_id, uint64 sample_time_unix_us, uint32 sequence,
uint32 protocol_version, float64[4] velocity_rad_s. Generated Python/C eşlemesi
ve 60-byte CDR roundtrip PASS. Canonical firmware mesaj hash'i aktarılan laptop
README'sinde aynı; MCU typesupport arşivi/yüklü imaj eşleşmesi doğrulanmadı.

Sonradan aktarılan salt-okunur kanıtlar:
- `wheel_command_checks/laptop_firmware_README.md` SHA256
  `edcbea8b530a7dee529298f1392c489ecb3881ab9d16dc3b23449251a0be2538`.
- `wheel_command_checks/laptop_pico_b_node_app.cpp` SHA256
  `bc93062c2434f837aa35638d3c53284a2bb75e402fb98b170b43e0694058b5ee`.
Bu yeni kopya eski PICO_B_MICROROS_CONTROL_NODE klasörünün yerine yazılmadı.

Kaynak sabitleri: protocol_version=1 (131), /pico_b/wheel_commands (284),
QoS BEST_EFFORT/VOLATILE/KEEP_LAST(1) (2241–2253). README final build:
WHEEL_COMMAND_SUBSCRIBER=1, CMD_VEL_SUBSCRIBER=0, SOURCE_TIME_MAPPING_VERIFIED=0.
Saat flag'i değiştirilmedi. Bu kayıt önceki PLAN'daki varsayımsal callback
beklentilerinin yerine, aktarılan gerçek kaynak davranışını koyar.

Hazırlanan araç: `tools/wheel_command_rx/publisher.py`, testleri
`test_publisher.py`; ayrıntılı komut/beklenti rehberi aynı dizindeki README.md.
Varsayılan offline; ROS node oluşturmaz. Canlı yayın yalnız explicit --publish
ve domain 0 ile; contract/hash/version/topic ve endpoint kontrolleri vardır.
Sabit test source_session_id=2026100101; başlangıç sequence=100, normalde artan
uint32; protocol_version=1; time.time_ns()//1000 kaynak Unix mikro-saniyesi.
Test değerleri FL/RL/FR/RR=[0.10,0.20,0.30,0.40] rad/s; fiziksel yön kalibrasyonu
değildir. Canlı normal yayın hedefi 20 Hz; bu görevde DDS rate ölçülmedi.

Senaryolar: normal, stop/stale (hiç yayın yok), duplicate (aynı seq), old
(son normal seq-1), her tekerlek için NaN/+Inf/-Inf, normal recovery. Duplicate
ve old güncel timestamp taşır; böylece sequence ret nedeni izole edilir. Suite
aynı session ile tek süreçte çalışır; yeniden süreç başlatılırsa --start-sequence
son kabul edilenden ileri seçilmeli. Agent restart veya session değişimi çözüm
olarak önerilmedi. Final zero gönderilmez; Ctrl-C/stale gerçekten yayını keser.

Doğru diagnostics anahtarları (3114–3203):
- pico_b/wheel_command: flRadS, rlRadS, frRadS, rrRadS, sequence, commandFresh.
- pico_b/wheel_freshness: localAgeMs, sourceAgeMs, localFresh, sourceTrusted,
  rejectCount, acceptedCount; ret nedeni status.message içindedir.
Normalde localFresh=true, localAgeMs<250, acceptedCount artar; mesajlar
RX_ONLY_TIME_UNVERIFIED / TIME_UNSYNCED. Bütün fazlarda sourceTrusted=false ve
commandFresh=false. sourceFresh iç alanı false fakat diagnostic key olarak
yayınlanmıyor. sourceAgeMs=0 trusted mapping yokken placeholder, gecikme ölçümü değil.

>=250 ms sessizlikte localFresh=false, aktif dört değer=0, son sequence korunur,
mesaj LOCAL_TIMEOUT; acceptedCount/rejectCount sırf sessizlikten artmaz.
Duplicate/old: SEQUENCE; NaN/Inf: NONFINITE; rejectCount artar, acceptedCount/
lastAcceptedRx/sequence ilerlemez; aktif değerler sıfırlanır. Sonraki geçerli
ileri sequence yerel kabulü geri getirir. Hata fazları 20 s; 25 sayfalık yaklaşık
12.5 s diagnostic turundan uzun. Sparse diagnostics kesin 250 ms latency ölçümü
yapmaz; BEST_EFFORT nedeniyle publisher sayısı=accepted sayısı varsayılmaz.

Yapılan kontroller: build PASS (`wheel_command_checks/build.log`); 6 offline
unit test PASS (`offline_tests.log`); version=1 ile 60 örnek CDR encode/decode
PASS (`verified_contract_offline.jsonl`, `validation.json`). İlk version=0
offline_fixture yalnız serializer fixture'ıydı, firmware sürümü iddiası değildir.
**Pico'ya yayın yapılmadı; rclpy node/publisher başlatılmadı.** Canlı firmware
callback testi, DDS 20 Hz ve motor TX ölçümü YAPILMADI. Firmware/Agent/library
değişmedi; upload veya hardware plugin eklenmedi. Hardware sürüş PASS yoktur.

Offline tekrar komutu (yeni log adı seçin):
```bash
cd /home/pi5/lessien_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/wheel_command_rx/test_publisher.py
python3 tools/wheel_command_rx/publisher.py --case suite --seconds 0.1 \
  --log wheel_command_checks/new_offline_run.jsonl
```

Yalnız gelecekteki yetkilendirilmiş cihaz testi için, BU GÖREVDE ÇALIŞTIRILMADI:
```bash
export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
python3 tools/wheel_command_rx/publisher.py --publish --case suite \
  --protocol-version 1 --session-id 2026100101 --start-sequence 100 \
  --seconds 20 --stale-seconds 20 \
  --firmware-contract wheel_command_checks/laptop_firmware_README.md \
  --log wheel_command_checks/live_suite.jsonl
```
Çalışan imajın receive-only/flag=0 olduğu ayrıca doğrulanmadan endpoint adı
kontrolü yeterli sayılmaz. Kullanıcı bu görevde yalnız hazırlık istedi.


## WheelCommand receive-only canlı cihaz testi — 2026-10-01

**Receive-only protokol davranışı PASS (32/32 faz); ölçülen RPM tümü-sıfır kontrolü
FAIL. Hardware sürüş PASS değildir.** Kullanıcı yeni receive-only UF2'nin
yüklendiğini bildirdi ve domain 0 canlı suite'ini açıkça yetkilendirdi. Firmware
imajı cihazdan okunup hash doğrulaması yapılmadı; yükleme bilgisi kullanıcı teyididir.

### Kapsam, ilk girişimler ve koşullu devam

Mevcut `tools/wheel_command_rx/README.md` komutu, değiştirilmemiş publisher.py ile
çalıştırıldı: --publish --case suite --protocol-version 1 --session-id 2026100101
--start-sequence 100 --values 0.10 0.20 0.30 0.40 --seconds 20 --stale-seconds 20.
Yalnız log yolu yeni kayıt dizinine yönlendirildi. Domain=0, discovery=SUBNET,
ROS_LOCALHOST_ONLY unset. Komut topic'i yalnız /pico_b/wheel_commands;
/cmd_vel yayını, hardware plugin, firmware değişikliği/upload veya saat güven
flag'i değişikliği yapılmadı. Agent başlatılmadı/durdurulmadı.

`capture_live.py` mevcut suite'in öncesi/sırası/sonrası RPM ve diagnostics kaydı
ve kontrollerini, `analyze_live.py` timestamp ile faz eşleştirmesini sağlar.
Her ikisi tools/wheel_command_rx altındadır; firmware kodu değildir.

İlk girişim `live_20261001_031351` diagnostics level bytes->JSON dönüşümünde,
7 sıfır RPM mesajından sonra, publisher başlamadan durdu; kayıt aracı düzeltildi.
İkinci girişim `live_20261001_031409`: 30.042 s salt-okunur baseline, 601 RPM
mesajı / 60 diagnostics. FL'de 4 adet -1, 597 adet 0; diğer motorlar 601 adet 0.
Motor hedefi iki diagnostics örneğinde 0; acceptedCount=0. Strict all-zero guard
nedeniyle publisher başlamadı. Bu -1 gözlemi wheel komutlarından önce gerçekleşti.

Otomatik onay incelemesi, nonzero RPM'e rağmen guard'ı gevşetip devam etmeyi
başlangıçta reddetti. Kullanıcı **“Koşullu devamı onaylıyorum”** yanıtıyla,
sıfır hedef/freshness kontrollerini koruyarak nonzero RPM'leri ayrı FAIL kaydıyla
raporlamak koşuluyla devamı açıkça yetkilendirdi. Bunun ardından RPM değerleri
maskelenmeden kaydedildi; ölçülmüş nonzero RPM normal veya arıza olarak
sınıflandırılmadı. commandRpm!=0, sourceTrusted/commandFresh!=false, eksik RPM
ve 2 s veri kesintisi kontrolleri korundu. Böyle bir ihlal ana suite'te görülmedi.

### Ana kayıt ve sonuçlar

Kanıt dizini: `wheel_command_checks/live_20261001_031602/`.
Başlangıç 2026-10-01 03:16:02.911 +03; bitiş 03:27:35.268 +03;
süre 692.358 s (baseline + suite + son 20 s). Publisher exit=0.
32 faz: 16 normal, 2 sessizlik, duplicate, eski sequence ve dört tekerleğin her
birinde NaN/+Inf/-Inf (12 nonfinite faz). Toplam 12000 mesaj yayınlandı.

| Kontrol | Sonuç / gözlem |
|---|---|
| Normal 20 Hz | PASS; host publish logunda normal faz hızları 19.99975–20.00025 Hz; her normal fazda localFresh=true |
| Normal snapshot | PASS; FL/RL/FR/RR=0.10/0.20/0.30/0.40 rad/s; localAgeMs=5–21; acceptedCount artışı |
| Stop / timeout, iki faz | PASS; yayın yokken LOCAL_TIMEOUT, localFresh=false, aktif dört değer=0 |
| Duplicate | PASS; SEQUENCE, rejectCount artışı; kabul edilmiş sequence ve acceptedCount ilerlemez |
| Eski sequence | PASS; SEQUENCE, rejectCount artışı; kabul edilmiş sequence ve acceptedCount ilerlemez |
| NaN, dört wheel ayrı | PASS; NONFINITE; tüm snapshot invalid/aktif değerler 0 |
| +Inf, dört wheel ayrı | PASS; NONFINITE; tüm snapshot invalid/aktif değerler 0 |
| -Inf, dört wheel ayrı | PASS; NONFINITE; tüm snapshot invalid/aktif değerler 0 |
| Normal recovery | PASS; her hata sonrası yeni geçerli örneklerde yerel kabul geri gelir |
| sourceTrusted / commandFresh | PASS; tüm alınan ilgili diagnostics örneklerinde false / false |
| Sayaç toplamı | acceptedCount 0→6400; rejectCount 0→5600, 12000 yayına karşılık gelir |
| Motor hedefi | 55 motor-summary örneğinin tamamında commandRpm=0; kaynak 0 RPM/0xFF invariantı değişmedi |
| Online / fault | 55 özette onlineMask=0x0F, faultMask=0x00 |
| Ölçülen RPM tamamı sıfır | **FAIL**; 13825 mesajın 98'inde en az bir -1; aşağıdaki dağılım |
| Agent | Başta/sonda aynı Agent PID 8548 çalışıyor; micro_ros ilk/son reconnectCount=0, üç publish-error sayacı=0 |

Ret/sessizlik fazlarındaki 26 freshness diagnostic örneğinde acceptedCount,
o ana kadar gönderilmiş normal mesaj sayısına eşit kaldı (`counter_validation.json`).
Nonfinite/duplicate/old alımının kabul sayacını artırdığı görülmedi. Son örnekte
LOCAL_TIMEOUT, localAgeMs=29020, acceptedCount=6400, rejectCount=5600.
Normalde RX_ONLY_TIME_UNVERIFIED / TIME_UNSYNCED ve WARN beklenen davranıştır;
birleşik freshness'in false kalması alım başarısızlığı olarak değerlendirilmedi.
sourceFresh ayrı diagnostic key olarak yayınlanmadığından cihazdan doğrudan
ölçülmüş olarak işaretlenmedi; sourceAgeMs=0 gecikme ölçümü sayılmadı.

### RPM dağılımı ve yorum sınırı

| Motor | 0 RPM örneği | -1 RPM örneği |
|---|---:|---:|
| ID1 FL | 13768 | 57 |
| ID2 RL | 13807 | 18 |
| ID3 FR | 13810 | 15 |
| ID4 RR | 13812 | 13 |

Örnek alım hızı 19.99899 Hz; maksimum gözlenen RPM mesaj aralığı 94.106 ms.
Bazı mesajlarda birden fazla motor -1 olduğundan motor başına toplam 103 nonzero
alan, 98 nonzero mesaja karşılık gelir. Başlangıç, tüm fazlar ve son sessizlik
boyunca ham RPM saklandı. Bu değerler sıfıra yuvarlanmadı, “normal gürültü” olarak
etiketlenmedi ve komutun motora aktarıldığının kanıtı sayılmadı. Fiziksel hareket
kamera/gözlemle bağımsız doğrulanmadı; hedef 0 ve ölçülen RPM ayrı sonuçlardır.

commandRpm=0 dönen diagnostics sayfalarındaki firmware beyanıdır; UART motor TX
logic-analyzer ile ölçülmedi. Kaynakta sabit brake=0xFF olması fiziksel bus frame
ölçümü değildir. 250 ms timeout sınırı kaynak sözleşmesidir; yaklaşık 12.5 s
diagnostic rotasyonu ile tam 250 ms durma gecikmesi ölçülemez. Fazlarda 1–3 ilgili
sayfa örneği var; sonuçlar bu gözlemlerin kapsamındadır. BEST_EFFORT kaydı sürekli
sıfır motor hedefi veya transient yokluğu garantisi değildir. Host publish rate,
Pico callback zamanlarının doğrudan rate ölçümü değildir; toplam alım sayaçları
beklenen 6400/5600 ayrımıyla tutarlıdır.

### Dosyalar ve korunan durum

- `rpm.jsonl`: 13825 ham RPM mesajı, host monotonic alım zamanı.
- `diagnostics.jsonl`: 1382 diagnostic mesajı, değerler/ret nedenleri ve alım zamanı.
- `publisher.jsonl`, `publisher_stdout.log`: gönderilen değerler, sequence,
  üretim zamanı, fazlar ve publisher çıkışı.
- `metadata.json`: ortam, exact command, baseline/final durum ve önce/sonra hashler.
- `analysis.json`, `analysis_summary.txt`, `counter_validation.json`: 32 fazın
  tek tek sonucu ve sayaç karşılaştırması. Analiz faz başındaki 0.5 s ve sondaki
  0.1 s geçiş aralığını dışlar; ham kayıt korunur.
- Önceki iki başarısız/başlamamış girişim dizini silinmedi.

Mesaj, laptop firmware kaynak/README ve publisher.py hashleri önce/sonra aynı.
Eski firmware kopyası da değiştirilmedi. Agent PID 8548 sonunda hâlâ çalışır;
yalnız test publisher/capture node'ları kapandı. Firmware/library/clock flag
üzerinde işlem yapılmadı. Bu sonuç receive-only kabul/ret protokolünü doğrular;
**sürüş, yön kalibrasyonu veya fiziksel güvenlik PASS'i değildir**. -1 RPM'in
nedeni ayrı inceleme konusu olarak açık kalır.

## -1 RPM Salt-Okunur Derin Analiz (B3A / canlı kayıt `live_20261001_031602`)

User confirmed that during the full ~11.5 minute receive-only test no visually observable wheel movement occurred.

This observation does not invalidate, mask, or classify the observed -1 RPM samples as normal.

### 1. Motor başına özet tablo (kanıt: `minus_one_readonly_analysis.json` → motor_summary)

| Motor (ID / Konum) | Toplam -1 RPM örneği | Burst (run) sayısı | Uzunluk-1 | Uzunluk-2 | Uzunluk-3 | Uzunluk-5 | Uzunluk-6 | En uzun gözlenen span | Sıfır çevresi bracket |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ID1 / FL  | 57 | 49 | 42 | 6  | 1 | — | — | 98.9 ms  | 94.2 – 199.9 ms |
| ID2 / RL  | 18 |  9 |  4 | 4  | — | — | 1 | 251.0 ms | 95.0 – 357.1 ms |
| ID3 / FR  | 15 |  6 |  3 | 1  | — | 2 | — | 204.1 ms | 99.0 – 296.1 ms |
| ID4 / RR  | 13 | 13 | 13 | —  | — | — | — | 0.0 ms* | 93.9 – 102.9 ms |

\* RR motorunda tüm -1 örnekleri tekil (ardışık 1 örnek), bu nedenle span 0.
Bracket, bir -1 örneğini çevreleyen 0-RPM örnekleri aralığıdır; gerçek olay süresi
için upper bound olarak kullanılabilir.

RPM örnekleme hızı: 19.99899 Hz (~50 ms/örnek).

### 2. Fazlara göre dağılım

Faz grubu özeti (phase_groups):

| Faz grubu | Örnek sayısı | ID1 FL -1 | ID2 RL -1 | ID3 FR -1 | ID4 RR -1 | Herhangi motorda -1 |
|---|---:|---:|---:|---:|---:|---:|
| baseline (yayın öncesi)         |  648 |  1 |  1 |  1 |  2 |  5 |
| normal (geçerli yayın)          | 6384 | 23 | 12 |  6 |  4 | 42 |
| stale / LOCAL_TIMEOUT / sessiz  | 1207 |  8 |  0 |  0 |  2 | 10 |
| rejected (toplam)               | 5586 | 25 |  5 |  8 |  5 | 41 |

Rejected alt kırılımı:

| Alt faz | ID1 FL | ID2 RL | ID3 FR | ID4 RR | Toplam |
|---|---:|---:|---:|---:|---:|
| duplicate (SEQUENCE)        |  0 |  0 |  0 |  1 |  1 |
| old (SEQUENCE, eski seq)    |  2 |  2 |  0 |  0 |  4 |
| nan  (NONFINITE / NaN)      | 15 |  1 |  7 |  0 | 23 |
| inf  (NONFINITE / +Inf)     |  3 |  0 |  0 |  0 |  3 |
| neg-inf (NONFINITE / -Inf)  |  5 |  2 |  1 |  4 | 12 |

**Sonuç:** `-1 RPM` örnekleri **baseline (yayın öncesi) dahil** beş farklı faz
grubunun tamamında gözlenmiştir. Yalnız tek bir command/reject fazına özgü
değildir; yayın olmaksızın salt-okunur fazda da oluşmuştur.

### 3. Burst başlangıç/bitiş ve süreleri (motor_diagnostics runs)

- En kısa olaylar: tek örnek (~50 ms bracket), tüm motorlarda çoğunluk.
- En uzun ardışık 3 örnek: FL, 98.9 ms gözlenen span.
- En uzun gözlenen ardışık 6 örnek: RL motorunda, 251.0 ms gözlenen span (tek
  olay, normal faz içinde).
- FR: 5'şer örnekli iki burst, 200 ms ve 204 ms.
- Tüm burst'ler 1-6 ardışık RPM örneği aralığında; hiçbir burst 12.5 s
  diagnostics rotasyonuna yakın veya onu aşan süre göstermemiştir.

### 4. Diagnostics alanlarıyla karşılaştırma

Önemli ön not: Motor diagnostics (currentA, sıcaklık, fault, errorHex, online)
**~12.5 saniyede bir** örneklenmektedir (motor_diagnostics.sample_gap_s_range:
12.496 – 12.505 s). -1 RPM burst'lerinin çoğu (< 260 ms) bu pencerenin en az
**48 katı daha kısa**dır. Dolayısıyla aşağıdaki karşılaştırmalar **eşzamanlı
ölçüm DEĞİLDİR**; yalnızca en yakın (~±6 s içinde) diagnostics örneğinin
olaydan farklı bir zamanda alındığı anlamında kullanılmalıdır.

Diagnostics grubu (motor_diagnostics, her motor için n=55 örnek):

| Motor | currentA aralığı | Sıcaklık aralığı | online | errorHex | Diagnostics RPM değerleri |
|---|---|---|---|---|---|
| ID1 FL | -0.89 – -0.71 A | 44 – 45 °C | true tüm örnek | 0x00 tüm örnek | "0" (54 örnek), "-1" (1 örnek) |
| ID2 RL | -0.45 – -0.30 A | 34 – 35 °C | true tüm örnek | 0x00 tüm örnek | "0" (55 örnek) |
| ID3 FR | -0.75 – -0.42 A | 40 – 40 °C | true tüm örnek | 0x00 tüm örnek | "0" (55 örnek) |
| ID4 RR | -0.78 – -0.39 A | 40 – 41 °C | true tüm örnek | 0x00 tüm örnek | "0" (55 örnek) |

Not: Diagnostics kanalında FL için yalnız 1 örnekte "-1" görüldü
(nearest_diagnostic_abs_offset_s ~6 ms yakınlıkta, bu da doğrudan eşleşme).
Diğer motorlarda diagnostics RPM alanı hiç "-1" olmamasına rağmen RPM topic
üzerinde -1 örnekleri kaydedilmiştir — bu, 20 Hz RPM topic ile ~0.08 Hz
diagnostics kanalının örnekleme uyumsuzluğundan kaynaklanır, bir tutarsızlık
değildir.

İletişim hata sayaçları (pico_b/motor_comm_12 ve _34 son örneği):

| Sayaç | M1 | M2 | M3 | M4 |
|---|---:|---:|---:|---:|
| crcErrorCount  | 0 | 0 | 0 | 0 |
| timeoutCount   | 0 | 0 | 0 | 0 |
| wrongIdCount   | 0 | 0 | 0 | 0 |

**İletişim katmanı sonucu**: 692 s test boyunca **sıfır CRC hatası, sıfır
timeout, sıfır wrong ID**. `-1 RPM` değerlerini taşıyan frame'ler
`validateMotorResponse` tarafından `MotorReplyResult::VALID` olarak kabul
edilmiştir (yani frame boyutu, CRC ve motor ID kontrollerinden geçmiştir).

### 5. Eski receive-only firmware dönemi karşılaştırması

Karşılaştırma için kullanılan kayıtlar:

- **Eski firmware kopyası (laptop_pico_b_node_app.cpp, wheel_command_checks/
  altında saklanan referans)**: RPM decode yolu (readDdsm115SignedBigEndian +
  motorRxFrame[4] offset) ile mevcut firmware **kelimesi kelimesine aynıdır**.
  Herhangi bir decode değişikliği YOKTUR.
- **live_20261001_031409 (suite başlamadan önceki 30 s baseline, eski
  firmware referansı ile aynı dönem)**: 601 RPM örneği; M1/FL'de 4 adet -1
  (0.666%), diğer motorlarda 0. Publisher bu kayıtta strict zero guard nedeniyle
  hiç başlamadı, yani 4 adet -1 **yayın öncesi, salt-okunur fazda** oluştu.
- **live_20261001_031351 (daha kısa, kesintiye uğramış girişim)**: 7 RPM örneği;
  -1 yok.
- **offline_fixture.jsonl / verified_contract_offline.jsonl**: Gerçek canlı
  RS485 kaydı değil (offline sözleşme fixture), karşılaştırma için uygun değil.

Sonuç: `-1 RPM` olgusu **eski firmware döneminde de (yayın başlamadan önceki
baseline kaydında) gözlenmiştir**; yalnızca yeni firmware veya yeni test
suite'i ile tanıtılmış bir durum DEĞİLDİR. FL motorunda her iki dönemde de en
yüksek sıklık görülür.

### 6. RPM decode yolu salt-okunur incelemesi

Kaynak: `PICO_B_MICROROS_CONTROL_NODE/pico_b_node_app.cpp`
(eski kopya: `wheel_command_checks/laptop_pico_b_node_app.cpp` — özdeş).

#### Çağrı zinciri
1. `finishMotorResponse` / `finishSpeedResponse` → `validateMotorResponse(...)`
2. **YALNIZCA** `result == MotorReplyResult::VALID` ise
   `storeValidMotorFeedback(...)` çağrılır.
3. `storeValidMotorFeedback` içinde (ID 1402–1415):
   - `status.mode = motorRxFrame[1];`
   - `currentRaw = readDdsm115SignedBigEndian(&motorRxFrame[2]);` → offset 2-3
   - `status.currentA = currentRaw * (8.0f / 32767.0f);`
   - **`status.measuredRpm = readDdsm115SignedBigEndian(&motorRxFrame[4]);`** → offset 4-5
   - `status.errorCode = motorRxFrame[8];`
4. RPM topic yayınlanırken (satır 2207-2208):
   `motorTelemetryFresh(i, nowMs) ? motorStatus[i].measuredRpm : 0;`
   (fresh değilse 0 kullanılır — ancak test boyunca onlineMask=0x0F,
   yani tüm motorlar her zaman fresh kabul edilir.)

#### `readDdsm115SignedBigEndian` (satır 1283-1286):
```cpp
int16_t readDdsm115SignedBigEndian(const uint8_t *data) {
  const uint16_t raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  return static_cast<int16_t>(raw);
}
```

- **Endianness**: Big-endian (high byte first, DDSM115 protokolü).
- **Ham byte → değer**: `data[4]` yüksek byte, `data[5]` düşük byte.
- **Tür dönüşümü**: Önce `uint16_t raw` olarak birleşir, sonra
  `static_cast<int16_t>(raw)` ile signed 16-bit tam sayıya dönülür. Bu dönüşüm
  **implementation-defined** görünse de; iki tümleyen (two's complement)
  mimaride (RP2040/ARM) `0x0000…0x7FFF` pozitif, `0x8000…0xFFFF` negatif
  anlamına gelir. Özellikle `raw == 0xFFFF` ise sonuç `int16_t(-1)` olur.
- **Ölçekleme**: RPM alanında doğrudan `int16_t` değeri kullanılır, herhangi
  bir `(X / 32767.0f)` ölçeği UYGULANMAZ (currentA'dan farklı olarak).

#### `-1` değerinin olası ham byte karşılığı
Yukarıdaki decode mantığı gereği, firmware tarafında loglanan `-1 RPM`
değerinin ham RS485 frame'deki karşılığı **`motorRxFrame[4] = 0xFF` ve
`motorRxFrame[5] = 0xFF`** (ikisi bir arada `0xFFFF`) olması gerekir. Bu
kanıtlanmıştır çünkü:

- CRC/ID/length kontrollerinden geçen frame'de `readDdsm115SignedBigEndian`
  fonksiyonunun `-1` döndürmesi için tek girdi yolu, ham 16-bit değerin
  `0xFFFF` olmasıdır.

ANCAK: `0xFFFF` aşağıdakilerden **hangisidir** — kanıtlanmamıştır ve varsayım
yapılmamalıdır:
- (a) Motor sürücüsünün gerçekten ölçtüğü ve döndürdüğü signed RPM değeri
  (yani -1 RPM),
- (b) Motor sürücüsünün bir sentinel / "data yok" / "durum hatası" bayrağı
  (protokol seviyesinde özel anlam),
- (c) Pico'nun UART/RS485 RX tarafındaki frame hizalama / drop / overrun
  kaynaklı bir yanlış byte sırası (CRC yine de geçiyor olsa bile — bu
  durumun CRC ile tutarlı olması düşük olasılıklı),
- (d) Başka bir protokol alanı ile çakışma veya yorumlama farkı.

### 7. Hipotez değerlendirmesi (kanıt olmadıkça KANITLANMADI)

Aşağıdaki hipotezlerden **hiçbiri kanıtlanmıştır** olarak sınıflandırılamaz.
Kanıt / karşıt kanıt durumları:

- **H1: Gerçek geri yönde 1 RPM mekanik hareket**
  - KANITLANMADI. Kullanıcı teyidi "gözle görülür hareket yok" şeklindedir;
    ancak 1 RPM (teker çevresi ~0.3 m/s çap ≈ 0.94 m/dk) çıplak gözle
    algılanamayabilir veya 250 ms'lik burst'ler boyunca ataletle oluşan
    mikro-hareketler kamerasız/güç ölçümü olmadan dışlanamaz. Mekanik
    sensör (IMU/enc) kanıtı yok.
- **H2: Encoder jitter veya sıfır-çevresi gürültüsü**
  - KANITLANMADI. Motor sürücüsü dahili encoder okumasında ±1 LSB
    yapabilir; ancak bunu doğrulamak için sürücü dökümanı veya ham encoder
    counter'ı gerekir.
- **H3: Command kaynaklı hareket**
  - KANITLANMADI; tersine bir miktar zayıf karşıt kanıt var:
    * `MOTOR_TARGET_RPM = 0` (static_assert ile zorlanmış),
    * `commandRpm=0` tüm 55 diagnostics örneği,
    * 12000 yayınlanan komutun 6400/5600 accepted/rejected ayrımı,
    * CRC/timeout/wrongId = 0.
    Ancak bu kanıtlar 20 Hz komut katmanıdır; motor sürücüsüne gönderilen
    RS485 TX frame'leri logic-analyzer ile doğrulanmamıştır.
- **H4: RS485 / CRC bozulması**
  - KANITLANMADI; doğrudan karşıt kanıt var: CRC hataları, timeout, wrongId,
    lengthError hepsi **0** tüm motorlarda. `validateMotorResponse` CRC geçen
    frame'lerde `storeValidMotorFeedback` çağırır. Dolayısıyla bozuk frame
    ölçülmüş -1 RPM kaynağı OLAMAZ (CRC geçmek zorunda).
- **H5: Motor firmware sentinel / özel değer**
  - KANITLANMADI. DDSM115 protokol dökümanı incelemede mevcut değil; 0xFFFF
    değerinin sürücü tarafından "ölçüm yok / initialize / hata" olarak mı
    kullanıldığı bilinmiyor.
- **H6: Firmware decode bug**
  - KANITLANMADI. `readDdsm115SignedBigEndian` big-endian + int16 cast,
    RP2040 two's complement üzerinde standart bir uygulamadır; aynı kod
    eski firmware döneminde de çalışıyor ve benzer -1 örnekleri vermekte.
    Ancak protokol RPM alanının aslında **unsigned** olup olmadığı veya
    `0xFFFF`'ın geçerli bir -1 RPM yerine "data invalid" anlamına
    gelip gelmediği bilinmediğinden, tamamen dışlanamaz.

### 8. Known / Unknown / Evidence Gap

**KNOWN (kanıtlananlar):**
- 11.5 dakikalık receive-only test boyunca gözle görülür tekerlek hareketi
  olmadığı kullanıcı tarafından teyit edilmiştir (yukarıda user confirmation
  metnine bakınız).
- `-1` örnekleri birden fazla fazda oluşmuştur: yayın öncesi (baseline),
  normal yayın, stale/sessizlik, duplicate, old-sequence, NaN, +Inf,
  -Inf. Tek bir command/ret tipine özgü DEĞİLDİR.
- Eski firmware referansı ile aynı dönemdeki (031409) baseline kaydında FL
  motorunda 4 adet `-1` gözlenmiştir; yayın başlamadan önceki salt-okunur
  dönemde de olgu mevcuttu.
- Diagnostics örnekleme hızı (~0.08 Hz / 12.5 s) 200–250 ms'lik burst'leri
  çözmek için yetersizdir; eşzamanlı teşhis verisi yok denebilir.
- CRC, timeout ve wrongId sayaçları tüm motorlarda 0'dır. `-1 RPM` içeren
  her frame CRC+ID+length doğrulamasından geçmiştir.
- Decode düzleminde `-1` ham karşılığı `motorRxFrame[4..5] = {0xFF, 0xFF}`
  olmak zorundadır.

**UNKNOWN (nedeni bilinmeyenler):**
- `-1`, motorun gerçek mekanik hareket sonucu mu ölçülen signed RPM
  değeridir?
- Motor sürücüsü firmware'inin raporladığı gerçek bir encoder ölçümü mü,
  yoksa dahili bir sentinel/protokol özel değeri mi (`0xFFFF` → "data
  unavailable" vb.)?
- Pico tarafında, protokol RPM alanının signed/unsigned yorumu doğru mudur?
  Yani `0xFFFF` motorda aslında "geçersiz" anlamına gelip Pico tarafından
  yanlışlıkla -1 RPM olarak mı yorumlanıyor?
- (Düşük olasılık) TX/RX frame hizalama, overrun veya benzeri UART katmanı
  problemi, ancak CRC garip bir şekilde hala geçiyor?

**EVIDENCE GAP (kanıt eksiklikleri):**
- DDSM115 protokol RPM alanının 0xFFFF için özel anlam tanımı (döküman yok).
- Herhangi bir -1 RPM olayı sırasında motor sürücüsüne gönderilen TX
  frame'lerinin ve alınan RX frame'lerinin HAM byte seviyesinde (logic
  analyzer veya Pico içi ek loglama ile) kaydı.
- IMU, harici encoder veya akım dalgasının yüksek hızlı (≥1 kHz) ölçümü;
  250 ms'lik burst içinde mikro-hareket veya akım değişikliği olup olmadığı.
- Diagnostics örneği ile RPM topic arası zaman damgası uyumsuzluğu nedeniyle,
  kısa burst'lerde currentA ve temperature değişimleri doğrulanamıyor.

### 9. Sonraki en küçük doğrulama adımı (firmware / command / clock flag değiştirmeden)

**Öneri**: Mevcut loglama araçları (`capture_live.py` + gerekirse salt-okunur
bir tanılama konusu yayınlayan ek bir ROS node) ile **her -1 RPM örneği için
ham RS485 response frame byte'larını (özellikle offset 0..9 + CRC byte'ı) ve
aynı anda decode edilen measuredRpm değerini eşzamanlı ve salt-okunur
olarak kaydedin**.

Gereksinimler / kısıtlar korunduğu halde yapılabilecekler:
- Firmware upload, motor komutu veya saat güven flag değişimi YOK.
- Sadece **okuma/yayın**: eğer firmware ham motor frame'lerini ayrı bir
  /debug topic'te yayınlama seçeneği zaten varsa (araştırılmalıdır) bunu
  kullanın; yoksa, `capture_live.py`'nın `rpm.jsonl`'ye ek olarak her
  -1 gördüğünde o anki diagnostics ve (varsa) UART/RS485 ham RX tamponu
  dökümünü tetiklemesini sağlayan salt-okunur host tarafı bir Python scripti
  eklenebilir.
- Mevcut loglarda ham frame verisi zaten yoksa, bir sonraki testte RPM konu
  yayınına ek olarak **ham `motorRxFrame[4]` ve `motorRxFrame[5]` byte'ları
  (0xFF 0xFF olup olmadığını doğrudan teyit için)** ayrı bir diagnostic
  değeri olarak yayınlanması — ancak bu firmware değişikliği gerektirdiğinden
  **ikinci aşamadır**. Birinci aşama tercihen host tarafı değişikliği ile
  sınırlandırılmalıdır.

### 10. Sonuç ve sınırlar

- Bu analiz receive-only protokol kabul/ret davranışı için ana suite
  kontrollerini (6400 accepted / 5600 rejected, LOCAL_TIMEOUT, NONFINITE,
  SEQUENCE davranışları) teyit eder.
- Hardware driving PASS **DEMEYİNİZ**.
- `-1 RPM` olaylarının kesin nedeni kanıt olmadan sınıflandırılmamıştır.
  Mekanik hareket, encoder jitter, protokol sentinel veya yorumlama farkı —
  dört olasılık da kanıt eksikliği nedeniyle AÇIK kalmaktadır.
- Kullanıcı gözlemi (hareket yok) güvenli bir koşul göstergesidir, ancak
  tek başına -1 örneklerini "yanlış alarm" veya "normal gürültü" olarak
  sınıflandırmak için yeterli değildir.

---

## Bölüm 11. Pico 2-B B4/B5A Firmware Upgrade — Source Changes + Static Review (2026-10-01)

### 11.1 Aktif workspace & source
- `Active workspace`: /home/pi5/lessien_ws  (pwd ile teyit)
- `Active source`   : /home/pi5/lessien_ws/PICO_B_MICROROS_CONTROL_NODE/pico_b_node_app.cpp
- `Spec & Tasks`    : /home/pi5/lessien_ws/.trae/specs/pico2b_firmware_B4_OLED_servo_B5A/
  - spec.md  (onaylı, 10 FR / 4 NFR / 11 AC)
  - tasks.md (onaylı, 7 task — T1–T4 kaynak bitti, T5–T7 BLOCKED)
- `User onayı`      : Verildi (given_files_content -> spec.md + tasks.md onay)

### 11.2 Non-Goals (korunan kısıtlar)
- `-1 RPM` konusu için FİRMARE DÜZELTME YOK (backlog: Bölüm 10).
- Motor sistemine dokunulmadı:
  - ID1=FL / ID2=RL / ID3=FR / ID4=RR,
  - `MOTOR_TARGET_RPM_BY_ID[0..3] == 0` static_assert korunur,
  - `MOTOR_BRAKE_ACTIVE_BYTE == 0xFF` (B5A cmd_vel sub durumu) korunur,
  - `motorDueUs[]` deadline dizisi ve `activeMotorIndex` DEĞİŞTİRİLMEDİ.
- Generic `PICO_B_ENABLE_SERVO` static_assert(==0) KORUNDU.
- `-1 RPM` publish filtresi / maskeleme YOK.

### 11.3 Source Changes Applied
#### (A) OLED B3B starvation / splash-stall
- `MIN_OLED_BUDGET_US = 1500` constexpr eklendi (L109).
- Global bayraklar eklendi (L618-L619):
  - `bool oledSplashShown = false;`
  - `uint32_t oledBudgetDroppedFrameCount = 0;`
- `setupOled()` splash guard (L3041):
  - `if (oledSplashShown) return;` → splash YALNIZCA boot sırasında 1 kez
  - `display.display()` sonrası `oledSplashShown = true;`
  - Blocking `delay()` / busy-wait YOK (grep onaylı, tüm dosyada 0 match).
- `updateOled()` bütçe gate (L2998-L3009):
  - `periodElapsed` başarısından sonra:
    `remaining = motorDueUs[activeMotorIndex] - micros();`
    `if (remaining < MIN_OLED_BUDGET_US) { ++drop; return; }`
  - `motorAllowsSlowTasks()` gate ÖNCEKİ gıpta ile korunur (L3015).
- `serviceOledManualPaging()` motor gate (L959-L964):
  - `#if PICO_B_ENABLE_MOTORS` altında: `if (!motorAllowsSlowTasks()) return;`
  - `++oledIgnoredInputCount;` sayacı korunur.
- `fillOledDiagnostic()` L2499:
  - index 5 key `"oledInitFailureCount"` → `"budgetDroppedFrames"` olarak
    değiştirildi (DIAG_VALUE_CAPACITY==6 kapasitesi korunur, boyut
    değiştirilmedi; `oledInitFailureCount` global sayaç korunur ama artık
    yayınlanmaz — init failure edge-case ise code-owner sonradan ekler).

#### (B) Servo merkezi config + mekanik limit (iki kat clamp)
- `CAM_SERVO_*` merkezi config sabitleri + static_assert zinciri eklendi
  (L252-L284):
  - `MIN/CENTER/MAX_ANGLE_DEG`, `HOME/FAILSAFE/MIN/MAX_PULSE_US`,
    `ABSOLUTE_MIN/MAX_US`, `UPDATE_PERIOD_MS`, `MAX_SLEW_DEG_P_S`,
    `SLEW_TENTHS_PER_UPDATE` — hepsi mevcut `SERVO_*` varsayımlarına
    bağlanır, compile-time static_assert ile min≤center≤max, absolute min≤
    yazılım min ≤ yazılım max ≤ absolute max zorlanır.
  - Kullanıcı uyarısı comment: mekanik limitler HARDWARE TEST ÖNCESİNDE
    KULLANICI ONAYI İLE kesinleştirilmeli.
- `CameraServoStatus` struct öncesi FİZİKSEL GERİ BİLDİRİM SINIRLAMASI
  comment bloğu (L441-L447):
  - RDS3235'te position feedback pini YOK → `currentAngleTenthsDeg` ve
    `currentPulseUs` SADECE yazılımsal tahmindir; mekanik yük/stall/dışarı
    itme durumunda reel fiziksel açıdan SAPABİLİR. Bu sınırlama AÇIK ve
    dokümante edildi.
- İki kat clamp mekanizması (L1196-L1203, L1233-L1241, L1309-L1317):
  1. Katman: `CAM_SERVO_MIN_PULSE_US` .. `CAM_SERVO_MAX_PULSE_US`
     (yazılım range)
  2. Katman: `CAM_SERVO_ABSOLUTE_MIN_US` .. `CAM_SERVO_ABSOLUTE_MAX_US`
     (Sürücü donanım limiti)
  Uygulanan yerler: `cameraServoAngleToPulseUs`, `initializeCameraServo`
  ilk home yazması, `cameraServoTask` periyodik output yazması.

#### (C) Servo non-blocking smooth hareket (constrain kalıbı)
- `cameraServoTask` L1294-L1320 yeniden yazıldı (eski if/else adımlama kaldırıldı):
  ```cpp
  error   = targetTenths - current;
  delta   = constrain(error, -CAM_SERVO_SLEW_TENTHS_PER_UPDATE,
                             +CAM_SERVO_SLEW_TENTHS_PER_UPDATE);
  current += delta;
  current  = constrain(current, MIN_ANGLE*10, MAX_ANGLE*10);
  ```
- `delay()` / uzun `for` / uzun `while` / busy-wait → YOK (grep 0 match).
- Update peryodu `CAM_SERVO_UPDATE_PERIOD_MS` (20 ms) ile kısıtlı.
- Motor 20 Hz scheduler, OLED, micro-ROS executor BLOKLANMAZ (task, 20 ms
  tick + constrain formülü → her tickte O(1), I2C yok).

#### (D) Servo startup (güvenli sıçramasız boot)
- `initializeCameraServo` L1212-L1247 yeniden yazıldı:
  - `targetAbsoluteAngleDeg = CAM_SERVO_CENTER_ANGLE_DEG`
  - `currentAngleTenthsDeg = CAM_SERVO_CENTER_ANGLE_DEG * 10`
  - `currentPulseUs = CAM_SERVO_HOME_PULSE_US` (yazılım durumu ilk home)
  - `attach(ABSOLUTE_MIN, ABSOLUTE_MAX, HOME_PULSE_US)` → Servo lib attach
    parametre olarak default pulse alır.
  - İLK `writeMicroseconds()` home pulse uzerinden İKİ KAT CLAMP ile yazılır.
  - Sonrasında SBUS sağlıklı input gelince, constrain kalıbı smooth hareket
    ile target'a ulaşır; aniden bilinmeyen hedefe sıçrama YOK.

#### (E) Servo failsafe (güvenlik semantiği KORUNUR)
- Mevcut semantik korunur: `inputHealthy == false` (SBUS frameLost,
  invalid, timeout vb.) İSE `relativeDeg = 0` → hedef CENTER → failsafe
  geçişi smooth slewi ile gerçekleşir (constrain kalıbı).
- Failsafe trigger sayacı `++cameraServoFailsafeCount` KORUNUR (L1271).
- Safety semantics: Eğer sistem tasarımı PWM'i doğrudan kapatmayı
  gerektiriyorsa bu smooth hareket uğruna DEĞİŞTİRİLMEDİ; şu an tasarım
  "failsafe → center pulse" modunda. PWM kapatma gerekiyorsa (üst seviye
  kararı) bu kod değişmeden önce kullanıcı kararı beklenir.

### 11.4 Static Review Sonuçları (kod değişiklikleri sonrası)
- `GetDiagnostics()` → 0 lint / type error.
- `grep "delay\\(|busy.*wait" pico_b_node_app.cpp` → 0 eşleşme.
- Motor static_assert zinciri KORUNDU (L484-L501):
  - MOTOR_COUNT==4,
  - MOTOR_SLOT_PERIOD_US 25000 / 16666 / 12500 (yani 10/15/20 Hz),
  - TARGET_RPM_BY_ID[0..3]==0,
  - MOTOR_CONFIGS[] id sırası 1=FL / 2=RL / 3=FR / 4=RR.
- `PICO_B_ENABLE_SERVO generic static_assert(==0)` KORUNDU (L77).
- `PICO_B_ENABLE_CAMERA_SERVO default 0` (compile-time -D ile 1 yapılır;
  özelleştirme flag'leri korunur).
- `motorDueUs[]`, `activeMotorIndex`, `motorAllowsSlowTasks()` → Bütün
  fonksiyonlar orijinal çalışma şeklini korur; sadece yeni eklenen gate'ler
  motor müsait değilse slow task'leri iptal eder.

### 11.5 Build / UF2 / SHA256 — BLOKELİ
Build toolchain sandbox erişimi nedeniyle KULLANILAMAZ.
- `which arduino-cli`      → NOT FOUND
- workspace + /opt + /usr/local + /usr/bin altında arduino-cli YOK
- `/home/yusuf` dizini bu sandbox içinde MEVCUT DEĞİL (README'deki
  `/home/yusuf/12_task/microros_rp2350_...` ve
  `/home/yusuf/.local/bin/arduino-cli` yollarına erişilemiyor).
- Dolayısıyla TASK 5 (clean build / UF2 / SHA256) ve TASK 6 (flash +
  hardware smoke test + servo küçük hareket testi) BLOKELİDİR.

#### Kullanıcının yapması gereken adımlar (A) — Elle build & hash paylaş
Lütfen fiziksel makinenizde (aktif toolchain + microros lib mevcut)
README.md'deki B4 ve B5A komutlarını --clean ile çalıştırıp sonuçları
paylaşın:
```bash
# B4 Final: motor 20 Hz + servo + relays, NO cmd_vel sub
arduino-cli --fqbn rp2040:rp2040:rpipico2 compile \
  --build-path /home/pi5/lessien_ws/build_pico_b_b4 \
  --build-cache-path /tmp/pico_b_cache --clean \
  --build-property "compiler.cpp.extra_flags=-DPICO_B_MOTOR_CYCLE_HZ=20 -DPICO_B_ENABLE_RELAYS=1 -DPICO_B_ENABLE_CAMERA_SERVO=1" \
  /home/pi5/lessien_ws/PICO_B_MICROROS_CONTROL_NODE

# B5A: cmd_vel subscriber açık, motor target compile-time 0, brake 0xFF
arduino-cli --fqbn rp2040:rp2040:rpipico2 compile \
  --build-path /home/pi5/lessien_ws/build_pico_b_b5a \
  --build-cache-path /tmp/pico_b_cache --clean \
  --build-property "compiler.cpp.extra_flags=-DPICO_B_MOTOR_CYCLE_HZ=20 -DPICO_B_ENABLE_RELAYS=1 -DPICO_B_ENABLE_CAMERA_SERVO=1 -DPICO_B_ENABLE_CMD_VEL_SUBSCRIBER=1" \
  /home/pi5/lessien_ws/PICO_B_MICROROS_CONTROL_NODE
```
Sonra her UF2 için:
```bash
UF2=/home/pi5/lessien_ws/build_pico_b_b4/PICO_B_MICROROS_CONTROL_NODE.ino.uf2
ls -la "$UF2" ; sha256sum "$UF2"
# B5A için tekrar et.
```
Paylaşılacak çıktı: PASS/FAIL, Flash usage %, RAM usage %, tam UF2 yolları,
SHA256 hash'leri. Sonra TASK 6 flash ve smoke testi ile devam ederiz.

#### Kullanıcının yapması gereken adımlar (B) — Alternatif: toolchain path sağla
Eğer build toolchain workspace içine kopyalanırsa veya /home/pi5/... altında
görünür bir yol ile erişilebilirse otomatik build denir. Fakat Arduino CLI
+ micro-ROS Arduino port'u büyük olduğundan (A) tercih edilir.

### 11.6 Hardware Validation Plan — Beklemede
Build + flash sonrası MOTOR KOMUTSUZ smoke test checklist (KULLANICI
onaylı uf2 sonrası uygulanır):
- Boot: normal boot, reset loop YOK.
- OLED: splash 1 kez (splash stall YOK), page update, manual page change,
  motor scheduler gecikmesi ≤500 us ortalama.
- micro-ROS: agent bağlantısı, pub/sub, agent restart sonrası Pico reset
  OLMADAN reconnect; reconnect'te splash TEKRAR YOK.
- Motor comm: online mask / fault mask / CRC error count / timeout count /
  wrongId count 0 veya beklenen değerler; failsafe semantics korunur.
- Servo: startup'ta ani sıçrama YOK, limit clamp aktif, output home
  (CENTER) güvenli durumda.
- Sonrasında: servo küçük hareket testi (home → +5° → home → -5° → home),
  clamp test (örneğin +100° clamp'in +MAX'a düşmesi, -100° clamp'in -MIN'a
  düşmesi). Hala MOTOR KOMUTU YOK.

### 11.7 Open issues
- `-1 RPM` (Bölüm 10 gibi AÇIK, no fix, no filter, only log).
- Build toolchain sandbox erişimi BLOK (T5, T6 bekliyor).
- RDS3235 mekanik limitleri (min/max/home) KULLANICI ONAYI bekliyor. Şu an
  repo varsayımı 550 / 1570 / 2450 us (0/90/180°) — fiziksel testten önce
  servo küçük hareket testinde (±5°) reaksiyonu görünce onaylanacak.

