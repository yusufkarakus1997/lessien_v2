#!/bin/bash
# =====================================================================
# OFFICIAL micro-ROS RP2350 Pico B SUB=2 Library Generation Wrapper
# KULLANILAN RESMI WORKFLOW: micro_ros_setup + colcon.meta + library_generation.sh
# SADECE FARK: colcon meta = colcon_rp2350_pico_b_sub2.meta (SUB=2, diğer limits SUB1 ile AYNI)
# OUTPUT KLASÖR: ${PORT}/microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy_pico_b_sub2
# Eski SUB1 library KORUNUR (ÜZERİNE YAZILMAZ)
# =====================================================================
set -e

UROS_WS=/home/pi5/lessien_ws/toolchains/uros_ws
STAGE=/home/pi5/lessien_ws/toolchains/pico_b_microros_stage
PORT=/home/pi5/lessien_ws/toolchains/microros_rp2350_jazzy_port
TOOLCHAIN_PREFIX=/home/pi5/.arduino15/packages/rp2040/tools/pqt-gcc/5.0.0-9576866/bin/arm-none-eabi-
TOOLCHAIN_CMAKE=${STAGE}/extras/library_generation/rp2350_arduino_pico_toolchain.cmake
SUB2_META=${STAGE}/extras/library_generation/colcon_rp2350_pico_b_sub2.meta
OUTPUT_NEW_LIB_DIR=${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b_sub2
TEMP_STAGE=$(mktemp -d /tmp/uros_stage_sub2_XXXXXX)
NEW_LIBRARY_NAME=micro_ros_arduino_pico_b_sub2
NEW_LIBRARY_VERSION=2.0.8-jazzy-picob-sub2

export TOOLCHAIN_PREFIX
export ROS_DISTRO=jazzy

echo "[GEN-SUB2] 1. Resmi /opt/ros/${ROS_DISTRO} sourced"
source /opt/ros/${ROS_DISTRO}/setup.bash 2>/dev/null || true

echo "[GEN-SUB2] 2. /uros_ws (micro_ros_setup colcon workspace) sourced"
cd ${UROS_WS}
if [ ! -f install/local_setup.bash ]; then
  echo "[GEN-SUB2] 2a. İLK KEZ: uros_ws colcon build (micro_ros_setup package)"
  colcon build --packages-up-to micro_ros_setup 2>&1 | tail -20
fi
source install/local_setup.bash 2>/dev/null || true

echo "[GEN-SUB2] 3. STAGE DIZININI TEMP KOPYALA (Üretim sırasında orijinal STAGE korunur)"
rsync -a ${STAGE}/ ${TEMP_STAGE}/
# Temp stage içindeki library.properties name güncelle (sonra output klasörüne kopyalarken orijinal korunur)
echo "[GEN-SUB2] 4. create_firmware_ws generate_lib (ROS2 + micro-ROS source code firmware/mcu_ws içine indir)"
cd ${TEMP_STAGE}
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib 2>&1 | tail -20

echo "[GEN-SUB2] 5. firmware/mcu_ws içine tf2_msgs + extra packages import (RESMI library_generation.sh İLE AYNI ADIMLAR)"
pushd firmware/mcu_ws > /dev/null
    git clone -b jazzy https://github.com/ros2/geometry2 --depth 1 2>&1 | tail -3
    cp -R geometry2/tf2_msgs ros2/tf2_msgs || true
    rm -rf geometry2 || true
    mkdir -p extra_packages
    pushd extra_packages > /dev/null
        if [ -d /project/extras/library_generation/extra_packages ]; then
          cp -R /project/extras/library_generation/extra_packages/* . || true
          [ -f extra_packages.repos ] && vcs import --input extra_packages.repos 2>&1 | tail -5 || true
        fi
    popd > /dev/null
popd > /dev/null

echo "[GEN-SUB2] 6. TEMIZLE: /project/src içinden eski C/C++ olmayan header dışı dosyaları sil (RESMI ADIM)"
find ${TEMP_STAGE}/src/ ! -name micro_ros_arduino.h ! -name "*.c" ! -name "*.cpp" ! -name "*.c.in" -delete 2>/dev/null || true

echo "[GEN-SUB2] 7. CROSS-COMPILE SUB2 libmicroros.a (Cortex-M33 softfp RP2350)"
rm -rf firmware/build 2>/dev/null || true
ros2 run micro_ros_setup build_firmware.sh ${TOOLCHAIN_CMAKE} ${SUB2_META} 2>&1 | tail -40

echo "[GEN-SUB2] 8. GENERATED INCLUDE + libmicroros.a KOPYA (resmi library_generation.sh ile BİREBİR AYNI)"
find firmware/build/include/ -name "*.c" -delete 2>/dev/null || true
cp -R firmware/build/include/* ${TEMP_STAGE}/src/ 2>&1 | tail -3
mkdir -p ${TEMP_STAGE}/src/cortex-m33/softfp
cp -R firmware/build/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/softfp/libmicroros.a
ls -la ${TEMP_STAGE}/src/cortex-m33/softfp/

echo "[GEN-SUB2] 9. Include path fix (RESMI ADIM - rsync package subdir)"
pushd firmware/mcu_ws > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list 2>/dev/null | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
popd > /dev/null
apt -y install rsync 2>&1 > /dev/null || true
for var in ${INCLUDE_ROS2_PACKAGES}; do
    rsync -r ${TEMP_STAGE}/src/${var}/${var}/* ${TEMP_STAGE}/src/${var}/ > /dev/null 2>&1 || true
    rm -rf ${TEMP_STAGE}/src/${var}/${var}/ > /dev/null 2>&1 || true
done

echo "[GEN-SUB2] 10. available_ros2_types + built_packages üret (RESMI ADIM)"
find firmware/mcu_ws/ros2 \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' > ${TEMP_STAGE}/available_ros2_types
find firmware/mcu_ws/extra_packages \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' >> ${TEMP_STAGE}/available_ros2_types
sort -o ${TEMP_STAGE}/available_ros2_types ${TEMP_STAGE}/available_ros2_types

cd firmware
echo "" > ${TEMP_STAGE}/built_packages
for f in $(find $(pwd) -name .git -type d); do pushd $f > /dev/null; echo $(git config --get remote.origin.url) $(git rev-parse HEAD) >> ${TEMP_STAGE}/built_packages; popd > /dev/null; done;
sort -o ${TEMP_STAGE}/built_packages ${TEMP_STAGE}/built_packages
cd ..

echo "[GEN-SUB2] 11. SON ADIM: OUTPUT YENİ KLASÖRE KOPYA (ESKI SUB1 KORUNUR)"
if [ -d ${OUTPUT_NEW_LIB_DIR} ]; then
  echo "Önceki SUB2 denemesi var, yedek al..."
  mv ${OUTPUT_NEW_LIB_DIR} ${OUTPUT_NEW_LIB_DIR}_old_$(date +%s)
fi
rsync -a ${TEMP_STAGE}/ ${OUTPUT_NEW_LIB_DIR}/

# SUB2 library.properties METADATA GÜNCELLE (isim + açıklama)
cat > ${OUTPUT_NEW_LIB_DIR}/library.properties <<EOF
name=${NEW_LIBRARY_NAME}
version=${NEW_LIBRARY_VERSION}
author=micro-ROS
maintainer=Eugenio Collado <eugeniocollado@eprosima.com>, Carlos Espinoza <carlosespinoza@eprosima.com>
sentence=micro-ROS Arduino library (Pico B separate SUB2 copy: SUB=2 SVC=0 CLNT=0)
paragraph=micro-ROS Arduino library, separate Pico B SUB2 copy: MAX_NODES=1 MAX_PUBLISHERS=10 MAX_SUBSCRIPTIONS=2 MAX_SERVICES=0 MAX_CLIENTS=0 MAX_HISTORY=2 CUSTOM_TRANSPORT=ON
url=https://github.com/micro-ROS/micro_ros_arduino
precompiled=true
category=Other
architectures=stm32,OpenCR,Teensyduino,samd,sam,mbed,esp32,mbed_portenta,mbed_giga,renesas_uno,mbed_opta,rp2040
EOF

echo "[GEN-SUB2] 12. Son SHA256:"
NEWLIB=$(find ${OUTPUT_NEW_LIB_DIR} -name "libmicroros.a" | head -1)
echo "NEW SUB2 libmicroros.a SHA256 = $(sha256sum ${NEWLIB} | awk '{print $1}')"
echo "OUTPUT DIR: ${OUTPUT_NEW_LIB_DIR}"
echo "TEMP Stage siliniyor: ${TEMP_STAGE}"
rm -rf ${TEMP_STAGE}
echo "[GEN-SUB2] ✅ RESMI OFFICIAL SUB=2 library generation TAMAMLANDI."
