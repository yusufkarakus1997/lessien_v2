# LESSIEN — URDF/Xacro ve izole ros2_control mock

Bu çalışma yalnız model ve **mock controller hazırlığıdır**. Donanım plugin'i,
Pico komut köprüsü, serial erişimi veya gerçek hareket yolu içermez.
Diferansiyel kinematik Pi 5 üzerindeki `diff_drive_controller` içindedir.
RDS3235 kamera servosudur, direksiyon değildir; bu modelde servo eklemi yoktur.

## Ölçüler ve açık varsayımlar

SI birimleri kullanılır. `wheel_separation` **toplam sağ-sol tekerlek merkez
mesafesi**, `wheelbase` **toplam ön-arka tekerlek merkez mesafesidir**.

| Xacro / launch parametresi | Varsayılan | Durum |
|---|---:|---|
| wheel_radius | 0.05035 m | Doğrulanmış; çap 100.7 mm |
| body_length | 0.32 m | Doğrulanmış |
| body_width | 0.195 m | Doğrulanmış |
| body_height | 0.10 m | Doğrulanmış |
| wheel_separation | 0.145 m | Kullanıcı ölçüm tanımından hesaplandı |
| wheelbase | 0.1757 m | Kullanıcı ölçüm tanımından hesaplandı |
| wheel_width | 0.045 m | Kullanıcı doğruladı |
| body_ground_clearance | 0.035 m | **UNVERIFIED**, gövde alt yüzeyi–zemin |
| body_mass | 2.0 kg | **UNVERIFIED** |
| wheel_mass | 0.15 kg / tekerlek | **UNVERIFIED** |

Kullanıcı ölçüm tanımını doğruladı: sağ-sol lastik iç yüzleri arası 0.10 m,
ön-arka lastik kenarları arası 0.075 m. Böylece merkez mesafeleri
0.10 + 0.045 = 0.145 m ve 0.075 + 0.1007 = 0.1757 m olur.
Yerden açıklık, kütleler ve kütle merkezi hâlâ doğrulanmadı. Katı kutu ve
silindir ataletleri varsayılan kütlelerden hesaplanır; fiziksel atalet ölçümü değildir.

**Basit box modelinde dört lastikle hacimsel çakışma vardır.** Gövde y aralığı
[-0.0975, +0.0975] m; sol lastik [0.050, 0.095], sağ lastik
[-0.095, -0.050] m. Lastiklerin tüm 0.045 m genişliği gövdenin y izdüşümündedir;
gövde dış lastik yüzünden her tarafta 0.0025 m taşar. Ön/arka lastik x
izdüşümleri [0.0375, 0.1382] ve [-0.1382, -0.0375] m ile gövde
[-0.16, 0.16] m içindedir. Mevcut UNVERIFIED açıklıkla gövde z=[0.035, 0.135],
lastikler z=[0, 0.1007] m: z izdüşüm örtüşmesi 0.0657 m'dir.
Silindirin merkez x kesitinde de kesişim bulunduğundan bu yalnız bounding-box
uyarısı değildir. Gerçek gövdede tekerlek oyukları/şekil farklılıkları olabilir;
katı box yaklaşımı bunları temsil etmez. Ölçüler değiştirilmedi. Gerçek çakışma
kararı için gövde şekli ve yerden açıklık ölçülmelidir. GenericSystem temas
simülasyonu yapmadığından mock PASS bu çakışmayı çözmez.

Dört sabit yönlü tekerleğin gerçek zemindeki skid-steer kayması mock ile modellenmez.

Varsayılanlar `src/lessien_description/config/geometry.yaml` içindedir.
Her iki launch tüm geometri parametrelerini kabul eder. Mock launch aynı
`wheel_separation` ve `wheel_radius` değerlerini Xacro'ya ve controller'a aktarır;
controller YAML'si tek başına kullanılmaz, bu iki gerekli parametre launch'ta eklenir.
Doğrudan Xacro da aynı YAML dosyasını göreli yoldan okur; sayısal varsayılanlar çoğaltılmaz.

## Model ve motor eşlemesi

x ileri, y sol, z yukarıdır. `base_footprint` zemin düzleminde, `base_link`
gövde merkezindedir. Varsayılan gövde merkezi z=0.085 m; tekerlek merkezleri
zeminden 0.05035 m yüksektir. Dört continuous eklem ekseni `0 1 0`;
pozitif tekerlek hızı ileri yuvarlanmaya karşılık gelir. Silindirlerin yerel z
eksenleri görsel/collision origin dönüşüyle y doğrultusuna çevrilir.

| Motor | Konum | Eklem |
|---|---|---|
| ID1 | FL | front_left_wheel_joint |
| ID2 | RL | rear_left_wheel_joint |
| ID3 | FR | front_right_wheel_joint |
| ID4 | RR | rear_right_wheel_joint |

ID tablosu belgelemedir; bu paket motor ID'lerine komut göndermez.
TF ağacı: `lessien_mock/odom -> lessien_mock/base_footprint ->
lessien_mock/base_link -> lessien_mock/{front_left,rear_left,front_right,rear_right}_wheel_link`.

## Kurulu Jazzy arayüzü ve resmi kaynaklar

2026-10-01 yerel package.xml doğrulaması:
`diff_drive_controller` ve `joint_state_broadcaster` **4.42.1**;
`controller_manager` ve `hardware_interface` **4.48.0**;
`robot_state_publisher` **3.3.4**, `xacro` **2.1.1**.

- [Resmi Jazzy diff_drive_controller belgesi](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html): `~/cmd_vel` tipi `geometry_msgs/msg/TwistStamped`; komut bileşenleri linear.x ve angular.z. `position_feedback=false` velocity feedback seçer; `open_loop=false` state geri bildirimiyle odometri üretir. `wheel_separation`, `wheel_radius`, `cmd_vel_timeout`, frame-prefix ve wheel-name parametreleri burada tanımlıdır.
- [Resmi 4.42.1 sürüm kaynağı](https://github.com/ros-controls/ros2_controllers/blob/4.42.1/diff_drive_controller/src/diff_drive_controller.cpp): topic, velocity arayüzleri ve mesaj header zamanına dayalı timeout doğrulandı. Kurulu C++ başlığı ve runtime subscriber da TwistStamped tipini doğruladı. `use_stamped_vel` gibi eski bir sürüm parametresi eklenmedi.
- [Resmi GenericSystem belgesi](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html): `calculate_dynamics=true` ile velocity komutundan position state entegrasyonu. Bu fizik/temas simülasyonu değildir.

`mock_components/GenericSystem`, `hardware_interface` paketindeki plugin'dir;
ayrı `mock_components` ROS paketi aranmaz. Dört joint'te velocity command,
velocity state ve görsel TF için position state vardır. Controller manager
50 Hz, odometri 50 Hz, komut timeout 0.5 s. Mock hız limitleri ±0.3 m/s ve
±1.0 rad/s; gerçek donanım güvenlik limitleri olarak doğrulanmadı.

## Build ve model doğrulama

Her terminalde yalnız mock çalışması için aşağıdaki ortamı kullanın;
Pico domain ayarını yapan `pico_b_env.sh` bu akışta source edilmez.

```bash
cd /home/pi5/lessien_ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR=/home/pi5/lessien_ws/mock_checks/ros_logs
colcon build --symlink-install --packages-select lessien_description lessien_bringup
source install/setup.bash
mkdir -p mock_checks
xacro src/lessien_description/urdf/lessien.urdf.xacro mock_control:=true > mock_checks/lessien_mock.urdf
check_urdf mock_checks/lessien_mock.urdf
```

## Mock çalıştırma ve test

İlk terminalde, yukarıdaki ortam ile:

```bash
ros2 launch lessien_bringup mock.launch.py
```

İkinci terminalde aynı setup, domain ve localhost ortamını ayarlayın:

```bash
cd /home/pi5/lessien_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR=/home/pi5/lessien_ws/mock_checks/ros_logs
ros2 control list_controllers -c /lessien_mock/controller_manager
ros2 control list_hardware_interfaces -c /lessien_mock/controller_manager
ros2 topic info /lessien_mock/diff_drive_controller/cmd_vel -v
ros2 run lessien_bringup verify_mock.py --output mock_checks/runtime.json
```

`verify_mock.py` önce GenericSystem, active controller'lar, URDF, parametreler ve
komut subscriber kimliğini kontrol eder; yalnız domain 77'de çalışır. Tek komut
hedefi `/lessien_mock/diff_drive_controller/cmd_vel` olup güncel zaman damgalı
TwistStamped yayınlar. İleri 0.1 m/s, yerinde sol dönüş 0.5 rad/s, yayın kesilince
timeout, joint velocity/position, odometri ve TF doğrulanır. Timeout testi
sıfır komut göndermeden yapılır; bitişte yalnız mock topicine sıfır gönderilir.
Pico'nun `/cmd_vel` veya başka motor komut topicine publish yapılmaz.

Mock topicleri:

- `/lessien_mock/diff_drive_controller/cmd_vel`: TwistStamped giriş
- `/lessien_mock/diff_drive_controller/odom`: Odometry
- `/lessien_mock/diff_drive_controller/cmd_vel_out`: sınırlandırılmış TwistStamped
- `/lessien_mock/joint_states`, `/lessien_mock/dynamic_joint_states`
- `/lessien_mock/robot_description`, `/lessien_mock/tf`, `/lessien_mock/tf_static`

Komut namespace'i sabittir; launch bir hardware plugin seçme seçeneği sunmaz.
Domain 77 ayrıca test terminal ortamıyla izole edilir; launch tek başına domain
zorlamaz. Test bitince mock launch terminalinde Ctrl-C kullanın. Micro-ROS Agent'ı
durdurmayın; bu akış Agent yönetmez.

Yalnız model publisher (mock launch ile aynı anda başlatmayın):

```bash
ros2 launch lessien_description description.launch.py
```

Bu launch robot_description ve sabit TF yayınlar; hareketli wheel TF için
joint_states kaynağı gerekir. Mock launch bunu joint_state_broadcaster ile sağlar.
Tüm geometri parametreleri değiştirilebilir; örneğin yalnız mock denemesi için
`ros2 launch lessien_bringup mock.launch.py wheel_separation:=0.26 wheelbase:=0.23`.
Bu alternatifler de **UNVERIFIED** ölçülerdir.

## Doğrulama sınırları

Sonuçlar `TEST_RESULTS.md` ve `mock_checks/` altında tutulur.
**Mock PASS, hardware PASS değildir.** Bu SSH oturumunda DISPLAY yok;
RViz açılmadı, görsel inceleme yapılmış sayılmaz. TF/model yapısı headless kontrol
edildi. Ekranlı bir oturumda RViz istenirse domain 77 ortamı ve
`/tf:=/lessien_mock/tf`, `/tf_static:=/lessien_mock/tf_static` remap'leri gerekir;
fixed frame `lessien_mock/odom`, RobotModel description topic
`/lessien_mock/robot_description` olmalıdır.

Pico firmware kopyası, eski motor_checks ve batarya sonrası kayıtlar değiştirilmedi
ve mock sonuçlarıyla birleştirilmedi. Gerçek donanım hareket testi yapılmadı.

## Pi mock + laptop RViz (iki cihaz)

**VISUAL NOT VERIFIED** — config ve model sayısal olarak kontrol edilse de
kullanıcı laptopta görüntüyü inceleyene kadar bu durum geçerlidir. Laptop
bağlantısı bu SSH oturumundan doğrulanmadı.

Hazır config: `lessien_description/rviz/lessien_mock.rviz`; RobotModel, TF ve
0.1 m hücreli Grid içerir. Fixed Frame `lessien_mock/odom`; RobotModel Description
Topic `/lessien_mock/robot_description` (Reliable / Transient Local), TF Prefix
`lessien_mock`. `rviz.launch.py` yalnız RViz açar; TF dinleyicisini
`/lessien_mock/tf` ve `/lessien_mock/tf_static` topiclerine yönlendirir. Mock
launch Pi'de çalışır, laptopta ikinci controller veya robot_state_publisher açılmaz.

### Ağ ve ortam

Aşağıdaki adımlar iki cihazda Ubuntu 24.04 / ROS 2 Jazzy ve laptopta çalışan
bir masaüstü/ekran oturumu varsayar. Laptopta RViz'i yerel terminalden açın;
Pi'ye SSH bağlantısında RViz açmanız gerekmez. Her iki cihaz aynı erişilebilir
LAN'da olmalı; DDS UDP/multicast iletişimi ve Wi-Fi istemcileri arası erişim
engellenmemelidir. Saatlerin senkron olması TF zaman sorunlarını önler.

**Her iki cihazda ROS_DOMAIN_ID=77.** Yukarıdaki tek cihaz test bölümünün
`LOCALHOST` ayarı bu iki cihaz akışı için uygun değildir: aşağıda `SUBNET`
kullanılır. Bu terminal değişiklikleri çalışan Micro-ROS Agent'ın ortamını
veya domainini değiştirmez; Agent'ı yeniden başlatmayın. `pico_b_env.sh` dosyasını
bu terminallerde source etmeyin.

### 1. Pi üzerinde mock

Pi terminalinde (önceki mock açıksa yalnız onun terminalinde Ctrl-C ile kapatın):

```bash
cd /home/pi5/lessien_ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export ROS_LOG_DIR=/home/pi5/lessien_ws/mock_checks/ros_logs
colcon build --symlink-install --packages-select lessien_description lessien_bringup
source install/setup.bash
ros2 launch lessien_bringup mock.launch.py
```

Yeni doğrulanmış ölçüler geometry.yaml üzerinden varsayılan olarak kullanılır. Pi'de LAN adresini ayrı terminalden `hostname -I` ile
öğrenip aşağıdaki laptop adımına girin.

### 2. Laptopa kaynak aktarımı ve yerel build

Model şu anda yalnız box/cylinder geometrilerinden oluşur; mesh, texture veya
`package://...` görsel kaynağı yoktur. URDF, Pi'den robot_description topic'iyle
alınır. Dolayısıyla çizim için yerel mesh gerekmez; aşağıdaki description paketi
aktarımı hazır config ve launch'ı sağlar. Bringup paketi laptopta gerekli değildir.
Pi'nin ARM64 `build/` veya `install/` dizinini laptopa kopyalamayın; laptop kendi
mimarisinde build etmelidir.

Laptopta Jazzy apt deposu ve temel ROS ortamı önceden kurulu olmalıdır. Eksik
paketler için (yalnız laptopta, gerekirse):

```bash
sudo apt update
sudo apt install ros-jazzy-rviz2 ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-rmw-fastrtps-cpp ros-jazzy-ament-cmake \
  python3-colcon-common-extensions python3-yaml
```

Laptop terminalinde:

```bash
read -r -p 'Pi LAN IP/hostname: ' LESSIEN_PI_HOST
mkdir -p ~/lessien_view_ws/src
scp -r "pi5@${LESSIEN_PI_HOST}:/home/pi5/lessien_ws/src/lessien_description" \
  ~/lessien_view_ws/src/
cd ~/lessien_view_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select lessien_description
source install/setup.bash
```

İleride `package://lessien_description/meshes/...` eklenirse mesh/texture dosyaları
paketin kaynak ağacında bulunmalı ve CMake install kurallarına eklenmelidir;
aynı `scp` aktarımından sonra laptopta yeniden build ve source yapılmalıdır.
Başka pakete ait `package://` URI varsa o kaynak paketi de laptopta kurulu/build
edilmiş olmalıdır. Mevcut primitive model için bu ek adım yoktur.

### 3. Laptopta keşif ve RViz

Laptopun yerel masaüstü terminalinde:

```bash
cd ~/lessien_view_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
ros2 topic list --no-daemon --spin-time 5 -t
ros2 topic echo /lessien_mock/robot_description std_msgs/msg/String \
  --qos-durability transient_local --qos-reliability reliable --once
ros2 launch lessien_description rviz.launch.py
```

Topic listesinde `/lessien_mock/robot_description`, `/lessien_mock/tf`,
`/lessien_mock/tf_static` ve `/lessien_mock/joint_states` beklenir.
Eşdeğer doğrudan RViz komutu (launch yerine):

```bash
rviz2 -d "$(ros2 pkg prefix --share lessien_description)/rviz/lessien_mock.rviz" \
  --ros-args -r __ns:=/lessien_mock \
  -r /tf:=/lessien_mock/tf -r /tf_static:=/lessien_mock/tf_static \
  -p use_sim_time:=false
```

Keşif gerçekleşmiyorsa iki tarafta domain, RMW, subnet, güvenlik duvarı ve eski
DDS XML profil/arayüz kısıtlarını kontrol edin. Multicast engelleniyorsa ilgili
cihaz terminalinde karşı cihazın erişilebilir LAN IP'sini girerek static peer
kullanabilirsiniz; ardından yalnız mock/RViz süreçlerini yeniden başlatın:

```bash
read -r -p 'Karsi cihazin LAN IP adresi: ' LESSIEN_PEER_IP
export ROS_STATIC_PEERS="$LESSIEN_PEER_IP"
```

[Resmi Jazzy keşif açıklaması](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Advanced/Improved-Dynamic-Discovery.rst)
SUBNET/LOCALHOST ve static peer davranışını açıklar. Aynı domain tek başına
ağ erişimini garanti etmez. RViz property adları ve topic QoS seçimi
[resmi Jazzy RobotModel kaynağıyla](https://github.com/ros2/rviz/blob/jazzy/rviz_default_plugins/src/rviz_default_plugins/displays/robot_model/robot_model_display.cpp)
kontrol edildi.

### Kullanılan tekerlek merkezleri

Yukarıdaki komutlarda `wheel_separation=0.145 m`, `wheelbase=0.1757 m`.
x=±wheelbase/2, y=±wheel_separation/2.
base_link, base_footprint'ten z=0.085 m yukarıdadır (0.035 m UNVERIFIED gövde
altı açıklığı + 0.10/2 m). wheel_radius=0.05035 m doğrulanmıştır.

| Motor / tekerlek | x (m) | y (m) | z, base_link'e göre (m) | z, base_footprint'e göre (m) |
|---|---:|---:|---:|---:|
| ID1 FL | +0.08785 | +0.07250 | −0.03465 | +0.05035 |
| ID2 RL | −0.08785 | +0.07250 | −0.03465 | +0.05035 |
| ID3 FR | +0.08785 | −0.07250 | −0.03465 | +0.05035 |
| ID4 RR | −0.08785 | −0.07250 | −0.03465 | +0.05035 |

Tablo gövdeye bağlı koordinatlardır; odom koordinatları mock hareketiyle değişir.
Parametre değişirse tablo da yeniden hesaplanmalıdır. Gerçek donanım hareket
testi yapılmadı. Önceki modelin Pi–laptop görüntüleme bağlantısı kullanıcı
teyidiyle VERIFIED; bu yeni geometrinin görsel incelemesi henüz yapılmadı.

Kullanıcı laptopta gövdeyi, dört tekerleği, zemin Grid'ini ve TF eksenlerini;
RobotModel/TF durumlarını ve tekerleklerin doğru taraflarda olduğunu kontrol
etmelidir. Kullanıcı teyidine kadar **VISUAL NOT VERIFIED**. Görüntünün düzgün
olması UNVERIFIED fiziksel ölçüleri doğrulamaz ve hardware PASS anlamına gelmez.
