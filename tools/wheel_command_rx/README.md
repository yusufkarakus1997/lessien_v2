# WheelCommand receive-only test aracı

2026-10-01 güncel canlı test: kullanıcı yetkisi ve nonzero RPM için koşullu devam
onayıyla suite çalıştırıldı. 32/32 receive-only faz PASS; strict all-zero ölçülen
RPM FAIL (-1 örnekleri). Kanıt: `wheel_command_checks/live_20261001_031602/`;
ayrıntılar kök AGENT_HANDOFF.md / TEST_RESULTS.md son bölümünde. Aşağıdaki
“yayın yapılmadı” ifadeleri önceki hazırlık aşamasının tarihsel kaydıdır.

Hazırlık tamamlandı; gerçek Pico'ya yayın yapılmadı. Varsayılan çalışma offline'dır:
ROS node oluşturmaz, yalnız generated typesupport ile CDR encode/decode yapar.
`--publish` eklenmesi domain 0'da gerçek yayın başlatır; aşağıdaki canlı komutlar
bu görevde ÇALIŞTIRILMADI ve sonraki cihaz testi içindir.

## Doğrulanan sözleşme

Mesaj SHA256:
`cb0405a45866b3cc63c502e0e726fe0fb00bdeaa425f7eb4874accd595472d6b`.
Aktarılan laptop README bu hash'i firmware canonical interface'i olarak bildiriyor.
Pi kaynak/installed mesaj hash'leri aynı, Python alan sırası ve generated C struct
kontrol edildi. MCU generated typesupport arşivi ve yüklü firmware imajı verilmedi;
binary/deployed eşleşmesi ayrıca doğrulanmış değildir.

```text
uint64 source_session_id
uint64 sample_time_unix_us
uint32 sequence
uint32 protocol_version
float64[4] velocity_rad_s
```

Yeni salt-okunur `wheel_command_checks/laptop_pico_b_node_app.cpp`:
version=1 (satır 131), topic=/pico_b/wheel_commands (284),
BEST_EFFORT/VOLATILE/KEEP_LAST(1) (2241–2253), sıra FL/RL/FR/RR.
Firmware README: SOURCE_TIME_MAPPING_VERIFIED=0, CMD_VEL_SUBSCRIBER=0,
WHEEL_COMMAND_SUBSCRIBER=1, motor hedefi 0 RPM ve brake 0xFF.
Bu araç firmware flag'lerini değiştirmez; çalışan imajı otomatik doğrulamaz.

Test session ID=2026100101, ilk sequence=100; bütün suite boyunca sabit session,
normal ve nonfinite örneklerde artan uint32 sequence. Bu ID ve hızlar protokol
sabiti değil seçilmiş test verileridir. FL/RL/FR/RR=[0.10,0.20,0.30,0.40] rad/s,
dört farklı değer sayesinde sıra görülebilir. Motor fiziksel işaretleri çıkarılmaz.
Kaynak üretim zamanı `time.time_ns() // 1000`, Unix mikro-saniyedir. Saat geri
giderse araç durur; timestamp uydurulmaz. Duplicate aynı sequence, old son normal
sequence-1 kullanır; ikisinde timestamp günceldir, böylece sequence hatası izole edilir.

Normal yayın 20 Hz (monotonic 50 ms takvim); gecikme sonrası catch-up burst yok.
Offline mod zaman beklemez ve 20 Hz runtime doğrulaması değildir. Her aşama
varsayılan 20 s; wheel build rotasyonu 25 sayfa ×0.5 s≈12.5 s olduğu için
bir tanılama turundan uzun tutulur. BEST_EFFORT kaybı mümkündür; gönderilen paket
sayısından accepted/reject sayısını birebir tahmin etmeyin. JSONL her örneği ve
faz sınırını kaydeder; NaN/Inf logda string, ROS mesajında IEEE nonfinite'dır.

## Build ve offline test (Pico'ya yayın yok)

```bash
cd /home/pi5/lessien_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select lessien_interfaces
source install/setup.bash
python3 tools/wheel_command_rx/test_publisher.py
python3 tools/wheel_command_rx/publisher.py --case suite --seconds 0.1 \
  --firmware-contract wheel_command_checks/laptop_firmware_README.md \
  --log wheel_command_checks/my_offline_run.jsonl
```

Log mevcutsa araç üzerine yazmadan durur; yeni dosya adı seçin. `--publish` yokken
rclpy.init/create_node/create_publisher çağrılmaz. Altı offline test PASS; version=1
ile suite 60 mesaj serialize etti, hiç yayınlamadı. CDR 60 byte (56-byte alan yükü
+ bu typesupport çıktısındaki encapsulation); wire/XRCE toplamı değildir.
İlk `offline_fixture.*` yalnız serialization çalışması için version=0 kullandı;
firmware geçerli sürümü iddiası değildir. Son kanıt `verified_contract_offline.*`.

## Sonraki cihaz testi için komutlar — bu görevde çalıştırılmadı

Önkoşul: çalışan imajın receive-only ve time-mapping flag=0 olduğu doğrulanmış
olmalı. Publisher'ın endpoint adı kontrolü bu imaj kanıtının yerini tutmaz.
Agent'ı durdurmayın, flash yapmayın. Aynı session ile yeni süreç başlatılırsa
`--start-sequence` son kabul edilenden ileri seçilmeli; aksi halde SEQUENCE
beklenir. Session'ı değiştirerek atlatmayın: firmware SESSION_CHANGE reddeder.
İlk sequence=100 yalnız yeni/uygun sequence durumunda geçerlidir. Tek suite
komutu süreçler arası sequence sorununu önler.

```bash
cd /home/pi5/lessien_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
# Salt-okunur diagnostics yakalama, ayrı terminalde aynı ortamla:
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray \
  --qos-reliability best_effort > wheel_command_checks/diagnostics_capture.yaml
```

Yalnız sonraki yetkilendirilmiş cihaz testinde (yaklaşık 10 dk 40 s, 32 aşama):

```bash
python3 tools/wheel_command_rx/publisher.py --publish --case suite \
  --protocol-version 1 --session-id 2026100101 --start-sequence 100 \
  --values 0.10 0.20 0.30 0.40 --seconds 20 --stale-seconds 20 \
  --firmware-contract wheel_command_checks/laptop_firmware_README.md \
  --log wheel_command_checks/live_suite.jsonl
```

Seçilebilir `--case`: normal, stale, duplicate, old, nan, inf, neg-inf, suite.
Tek hata senaryosu önce normal baseline üretir. Tek nonfinite senaryoda
`--wheel-index 0|1|2|3` FL/RL/FR/RR seçer; suite hepsini dener. Suite her hata
öncesi normal recovery fazı içerir. Normal/stale örneği için yukarıdaki komutta
`--case stale` ve yeni log adı kullanılır. Ctrl-C de hemen yayını keser; final zero
veya keepalive gönderilmez. Stale aşamasında hiçbir mesaj üretilmez.

Canlı mod explicit domain=0, doğrulanmış sözleşme/README/source hash'i ve doğru
version/topic ister. Başka publisher varsa ya da tek Pico B WheelCommand
subscriber görünmüyorsa başlamaz; endpoint sayısı değişirse durur. Bu kontroller
kimlik doğrulama veya firmware güvenlik kanıtı değildir. Hardware plugin yoktur.

## Beklenen diagnostics (kaynak doğrulaması; cihazda henüz test edilmedi)

`pico_b/wheel_command`: flRadS, rlRadS, frRadS, rrRadS, sequence, commandFresh.
`pico_b/wheel_freshness`: localAgeMs, sourceAgeMs, localFresh, sourceTrusted,
rejectCount, acceptedCount. Ret nedeni ikinci sayfanın `message` alanıdır.

| Aşama | Beklenen kaynak davranışı |
|---|---|
| Normal 20 Hz | Dört değer 0.10/0.20/0.30/0.40; acceptedCount artar; sequence ilerler; localFresh=true, localAgeMs<250; mesaj RX_ONLY_TIME_UNVERIFIED / TIME_UNSYNCED |
| Stop/stale | Son kabulden >=250 ms sonra localFresh=false, aktif dört değer=0, sequence son kabulde kalır; RX_ONLY_INVALID / LOCAL_TIMEOUT; acceptedCount ve rejectCount sessizlikte artmaz |
| Duplicate / old | rejectCount artar; acceptedCount ve kabul edilmiş sequence ilerlemez; aktif değerler sıfır, localFresh=false; ret SEQUENCE |
| NaN/+Inf/-Inf | Herhangi bir wheel nonfinite ise tüm mesaj reddedilir; rejectCount artar, acceptedCount ve kabul edilmiş sequence ilerlemez; dört aktif değer=0, localFresh=false; ret NONFINITE |
| Yeni geçerli recovery | İleri sequence, aynı session ve geri gitmeyen timestamp ile yeniden localFresh=true, acceptedCount artar |

**Bütün fazlarda sourceTrusted=false, commandFresh=false beklenir.** SourceFresh
firmware iç alanıdır; mevcut diagnostics'te ayrı key olarak yayınlanmıyor.
Flag=0 nedeniyle iç sourceFresh=false kalır. `sourceAgeMs=0` bu build'de ölçülmüş
sıfır gecikme değildir: trusted mapping yokken başlangıç değeri olarak yazılır.
Normal alımda WARN/TIME_UNSYNCED beklenmesi alım testi başarısızlığı değildir.
Yerel başarı localFresh/localAgeMs/acceptedCount üzerinden değerlendirilir.

Kaynak callback (813–900): null/version/session-zero/time-zero/nonfinite/session
change/sequence/time-rewind kontrolleri; source age/future kontrolleri yalnız
sourceTimeTrusted true ise. Flag=0 iken Unix zaman alanı taşınsa da source freshness
kanıtlanmaz. Sequence kontrolü uint32 delta>0 ve delta<2^31, wrap kabul edilir.
Reject aktif değerleri sıfırlar fakat session/lastAcceptedRx/sequence'i korur;
reddedilen mesajlar alım timeout'unu yenilemez. Local timeout >=250 ms (3677–3681).

Tanılama sayfaları farklı anlarda gelir; transient ret nedenini görmek için hata
fazını tam rotasyondan uzun tutun ve sayaç deltalarını saklayın. Bu sparse log
250 ms timeout gecikmesini hassas ölçmez; yalnız sonraki stale durumunu gösterir.
Motor hedefi 0/brake FF kaynak invariantıdır; cihazdaki TX bu görevde ölçülmedi.
Hardware sürüş PASS yoktur.
