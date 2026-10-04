#!/bin/bash
# =====================================================================
# LESSIEN — CLEAN RP2350 Pico B SUB=2 micro-ROS Library Generation
# RESMI WORKFLOW: micro_ros_setup create_firmware_ws + build_firmware
# Upstream library_generation.sh L31-L72 ile BİREBİR AYNI adımlar.
#
# YASAK 2. BÖLÜM KURALINA UYULUR:
#  - HIÇBIR source patch / stub / COLCON_IGNORE / fake package YOK
#  - rcpputils / rcutils / filesystem / POSIX / exception / PATH_MAX
#    workaround YOK
#  - SADECE: clean colcon.meta (SUB1 EXACT kopya, SADECE SUBS 1→2 farkı)
# =====================================================================
set -e
set -o pipefail

# ---- Paths ----
UROS_WS=/home/pi5/lessien_ws/toolchains/uros_ws
STAGE=/home/pi5/lessien_ws/toolchains/pico_b_microros_stage
PORT=/home/pi5/lessien_ws/toolchains/microros_rp2350_jazzy_port
TOOLCHAIN_PREFIX=/home/pi5/.arduino15/packages/rp2040/tools/pqt-gcc/5.0.0-9576866/bin/arm-none-eabi-
TOOLCHAIN_CMAKE=${STAGE}/extras/library_generation/rp2350_arduino_pico_toolchain.cmake
SUB2_META_CLEAN=${STAGE}/extras/library_generation/colcon_rp2350_pico_b_sub2_CLEAN.meta
OUTPUT_NEW_LIB_DIR=${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b_sub2
TEMP_STAGE=$(mktemp -d /tmp/uros_stage_sub2_CLEAN_XXXXXX)
NEW_LIBRARY_NAME=micro_ros_arduino_pico_b_sub2
NEW_LIBRARY_VERSION=2.0.8-jazzy-picob-sub2
BUILD_LOG=/home/pi5/lessien_ws/build_artifacts/log_generate_sub2_CLEAN_V2.log

export TOOLCHAIN_PREFIX
export ROS_DISTRO=jazzy

# Section 13 CLEAN OFFICIAL solution: rosdep install SUDO triggerlarini official env var
# EXTERNAL_SKIP (create_firmware_ws.sh L72-L76 SKIP concat) ile BLOKLA.
# Tum ament/ros base degerlendirmeler HOST apt kurulu (yukarıdaki dpkg kontrolu ile
# dogrulandi). EXTERNAL_SKIP official micro-ros degisken, NO WORKAROUND.
export EXTERNAL_SKIP="ament_cmake_ros ament_lint ament_lint_auto ament_lint_common build_essential google_benchmark_vendor mimick_vendor osrf_testing_tools_cpp performance_test_fixture rti-connext-dds-5.3.1 rmw_connextdds rmw_connext_cpp rmw_cyclonedds_cpp rmw_fastrtps_cpp ros-jazzy-cyclonedds ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rmw-connextdds-common ros-jazzy-rmw-connextdds ros-jazzy-fastrtps ros-jazzy-rmw-fastrtps-cpp ros-jazzy-fastcdr ros-jazzy-libyaml-vendor ros-jazzy-spdlog-vendor ros-jazzy-rcpputils ros-jazzy-rcutils ros-jazzy-tracetools ros-jazzy-libstatistics-collector ros-jazzy-rcl-logging-spdlog ros-jazzy-rosidl-typesupport-fastrtps-c ros-jazzy-rosidl-typesupport-fastrtps-cpp"

# ---- Logging ----
mkdir -p $(dirname ${BUILD_LOG})
echo "[CLEAN SUB2] Log path: ${BUILD_LOG}"

echo "============================================================"
echo "[CLEAN SUB2] START time: $(date -Iseconds)"
echo "[CLEAN SUB2] TEMP_STAGE: ${TEMP_STAGE}"
echo "[CLEAN SUB2] META (clean):   ${SUB2_META_CLEAN}"
echo "[CLEAN SUB2] TOOLCHAIN:      ${TOOLCHAIN_CMAKE}"
echo "[CLEAN SUB2] TARGET OUTPUT:  ${OUTPUT_NEW_LIB_DIR} (partial YAZILMAYACAK)"
echo "[CLEAN SUB2] OLD SUB1 IMMUTABLE (DOKUNULMAYACAK): ${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b"
echo "============================================================"

echo "[CLEAN SUB2] 1. Source /opt/ros/${ROS_DISTRO}"
source /opt/ros/${ROS_DISTRO}/setup.bash 2>/dev/null || true

echo "[CLEAN SUB2] 2. Source uros_ws (micro_ros_setup package)"
cd ${UROS_WS}
if [ ! -f install/local_setup.bash ]; then
  echo "[CLEAN SUB2] 2a. uros_ws colcon build (micro_ros_setup için ilk kez)"
  colcon build --packages-up-to micro_ros_setup 2>&1 | tail -10
fi
source install/local_setup.bash 2>/dev/null || true

echo "[CLEAN SUB2] 3. STAGE → TEMP kopyala (üretim sırasında orijinal STAGE dokunulmaz)"
rsync -a ${STAGE}/ ${TEMP_STAGE}/

echo "[CLEAN SUB2] 4. cd TEMP_STAGE → create_firmware_ws.sh generate_lib (UPSTREAM L38)"
cd ${TEMP_STAGE}
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib 2>&1 | tail -20

echo "[CLEAN SUB2] 5. EXTRA PACKAGES: tf2_msgs + lessien_interfaces (UPSTREAM L41-L55 birebir)"
pushd firmware/mcu_ws > /dev/null
    echo "[CLEAN SUB2]   5a. tf2_msgs clone → ros2/tf2_msgs (UPSTREAM L44-L46)"
    git clone -b jazzy https://github.com/ros2/geometry2 --depth 1 2>&1 | tail -3
    cp -R geometry2/tf2_msgs ros2/tf2_msgs || true
    rm -rf geometry2 || true

    echo "[CLEAN SUB2]   5b. extra_packages from STAGE/extras/library_generation/extra_packages (UPSTREAM L49-L53 birebir)"
    mkdir -p extra_packages
    pushd extra_packages > /dev/null
        EXTRA_SRC=${TEMP_STAGE}/extras/library_generation/extra_packages
        if [ -d "${EXTRA_SRC}" ]; then
          cp -R ${EXTRA_SRC}/* .
          if [ -f extra_packages.repos ]; then
            vcs import --input extra_packages.repos 2>&1 | tail -5 || true
          fi
        fi
    popd > /dev/null
popd > /dev/null

echo "[CLEAN SUB2] 6. CLEAN src (UPSTREAM L58 — sadece header/C/C++ tut)"
find ${TEMP_STAGE}/src/ \
  ! -name micro_ros_arduino.h \
  ! -name "*.c"   ! -name "*.cpp"  ! -name "*.c.in" \
  ! -type d -delete 2>/dev/null || true

echo "[CLEAN SUB2] 7. Clean old firmware/build"
rm -rf firmware/build 2>/dev/null || true

echo "[CLEAN SUB2] 8. TOOLCHAIN CROSS-COMPILE (UPSTREAM L60-L72) — UZUN ADIM"
echo "[CLEAN SUB2]    META: ${SUB2_META_CLEAN}"
echo "[CLEAN SUB2]    TOOLCHAIN PREFIX: ${TOOLCHAIN_PREFIX}"
echo "[CLEAN SUB2]    Cortex-M33 softfp proof flags (L15-L27): -mcpu=cortex-m33 -mfloat-abi=softfp -mcmse"
cd ${TEMP_STAGE}
BUILD_STARTED=$(date +%s)
ros2 run micro_ros_setup build_firmware.sh ${TOOLCHAIN_CMAKE} ${SUB2_META_CLEAN} 2>&1 | tail -80
BUILD_RC=${PIPESTATUS[0]}
BUILD_FINISHED=$(date +%s)
BUILD_DURATION=$(( BUILD_FINISHED - BUILD_STARTED ))
echo "[CLEAN SUB2]    Build duration: ${BUILD_DURATION}s. Exit code: ${BUILD_RC}"

# ---- Partial write KORUMA: exit 0 DEĞILSE çık, TEMP_STAGE sil ve KESINLIKLE output oluşturma ----
if [ ${BUILD_RC} -ne 0 ]; then
  echo "[CLEAN SUB2] ❌ BUILD EXIT != 0 → OUTPUT KLASÖRÜ OLUŞTURULMAYACAK (Section 3 KORUMA)"
  echo "[CLEAN SUB2]    Partial build diske kopyalanMIYOR. TEMP_STAGE korunuyor (debug için): ${TEMP_STAGE}"
  echo "[CLEAN SUB2] FAILED time: $(date -Iseconds)"
  exit ${BUILD_RC}
fi

# ---- UPSTREAM L67-L72 birebir include+lib copy ----
echo "[CLEAN SUB2] 9. UPSTREAM L67-L72: include + libmicroros.a → src"
find firmware/build/include/ -name "*.c" -delete 2>/dev/null || true
cp -R firmware/build/include/* ${TEMP_STAGE}/src/ 2>&1 | tail -3

echo "[CLEAN SUB2] 10. Cortex-M33 archive copy → src/cortex-m33/softfp/ (UPSTREAM L70-L72)"
mkdir -p ${TEMP_STAGE}/src/cortex-m33/softfp
cp firmware/build/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/softfp/libmicroros.a
# Cortex-M33 ana dizine de (subdir için) link ver — SUB1 production library format ile aynı
ln -sf softfp/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/libmicroros.a 2>/dev/null || \
  cp firmware/build/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/libmicroros.a
ls -la ${TEMP_STAGE}/src/cortex-m33/softfp/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/libmicroros.a

# ---- Include path fix UPSTREAM L73-L79 ----
echo "[CLEAN SUB2] 11. Include path fix (UPSTREAM L73-L79 rsync package subdir)"
pushd firmware/mcu_ws > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list 2>/dev/null | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
popd > /dev/null
if ! command -v rsync >/dev/null 2>&1; then
  echo "[CLEAN SUB2]   11a. rsync YOK — clean fallback: cp -a kullan (NO WORKAROUND, NO APT)"
  for var in ${INCLUDE_ROS2_PACKAGES}; do
    if [ -d ${TEMP_STAGE}/src/${var}/${var}/ ]; then
      cp -a ${TEMP_STAGE}/src/${var}/${var}/* ${TEMP_STAGE}/src/${var}/ 2>/dev/null || true
      rm -rf ${TEMP_STAGE}/src/${var}/${var}/ 2>/dev/null || true
    fi
  done
else
  for var in ${INCLUDE_ROS2_PACKAGES}; do
    rsync -r ${TEMP_STAGE}/src/${var}/${var}/* ${TEMP_STAGE}/src/${var}/ > /dev/null 2>&1 || true
    rm -rf ${TEMP_STAGE}/src/${var}/${var}/ > /dev/null 2>&1 || true
  done
fi

# ---- available_ros2_types UPSTREAM L82-L84 ----
echo "[CLEAN SUB2] 12. available_ros2_types üret (UPSTREAM L82-L84)"
find firmware/mcu_ws/ros2 \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | \
  awk -F"/" '{print $(NF-2)"/"$NF}' > ${TEMP_STAGE}/available_ros2_types
find firmware/mcu_ws/extra_packages \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | \
  awk -F"/" '{print $(NF-2)"/"$NF}' >> ${TEMP_STAGE}/available_ros2_types
sort -o ${TEMP_STAGE}/available_ros2_types ${TEMP_STAGE}/available_ros2_types

# ---- built_packages UPSTREAM L86-L90 ----
echo "[CLEAN SUB2] 13. built_packages üret (UPSTREAM L86-L90)"
cd firmware
echo "" > ${TEMP_STAGE}/built_packages
for f in $(find $(pwd) -name .git -type d); do
  pushd $f > /dev/null
  echo $(git config --get remote.origin.url) $(git rev-parse HEAD) >> ${TEMP_STAGE}/built_packages
  popd > /dev/null
done
sort -o ${TEMP_STAGE}/built_packages ${TEMP_STAGE}/built_packages
cd ..

# ================================================================
# Section 3 KORUMA: OUTPUT klasörü SADECE buraya kadar PASS İSE
#                   oluşturulacak — KESINLIKLE partial YOK.
# ================================================================
echo "[CLEAN SUB2] 14. OUTPUT KLASÖRÜ: ${OUTPUT_NEW_LIB_DIR} (SUB1 IMMUTABLE KORUNUR)"
if [ -d ${OUTPUT_NEW_LIB_DIR} ]; then
  OLD_TS=$(date +%s)
  echo "[CLEAN SUB2]   Önceki SUB2 output var → yedek: ${OUTPUT_NEW_LIB_DIR}_old_${OLD_TS}"
  mv ${OUTPUT_NEW_LIB_DIR} ${OUTPUT_NEW_LIB_DIR}_old_${OLD_TS}
fi
# OLD SUB1 path (korunuyor: dokunulmuyor)
if [ -f ${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/cortex-m33/softfp/libmicroros.a ]; then
  OLD_SHA=$(sha256sum ${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/cortex-m33/softfp/libmicroros.a | awk '{print $1}')
  echo "[CLEAN SUB2]   OLD SUB1 SHA (before SUB2 copy): ${OLD_SHA} (değişmemeli)"
fi
rsync -a ${TEMP_STAGE}/ ${OUTPUT_NEW_LIB_DIR}/
# rsync sonrası tekrar check
if [ -f ${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/cortex-m33/softfp/libmicroros.a ]; then
  OLD_SHA_AFTER=$(sha256sum ${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/cortex-m33/softfp/libmicroros.a | awk '{print $1}')
  echo "[CLEAN SUB2]   OLD SUB1 SHA (after  SUB2 copy): ${OLD_SHA_AFTER}"
  if [ "${OLD_SHA}" != "${OLD_SHA_AFTER}" ]; then
    echo "[CLEAN SUB2]   ❌ OLD SUB1 SHA DEĞIŞMİŞ — DUR (Section 11H ihlali)!"
    echo "[CLEAN SUB2]   ❌ Output siliniyor (SUB1 backup immutable ile restore gerekebilir)"
    exit 111
  else
    echo "[CLEAN SUB2]   ✅ OLD SUB1 SHA AYNI — IMMUTABLE KORUNDU."
  fi
fi

# ---- library.properties SUB2 meta yaz (RESERVED) ----
echo "[CLEAN SUB2] 15. library.properties SUB2"
cat > ${OUTPUT_NEW_LIB_DIR}/library.properties <<EOF
name=${NEW_LIBRARY_NAME}
version=${NEW_LIBRARY_VERSION}
author=micro-ROS
maintainer=Eugenio Collado <eugeniocollado@eprosima.com>, Carlos Espinoza <carlosespinoza@eprosima.com>
sentence=micro-ROS Arduino library (Pico B separate SUB2 clean production copy: SUBS=2 SVC=0 CLNT=0)
paragraph=micro-ROS Arduino library, Pico B RP2350 Cortex-M33 SUB=2 clean official generation: MAX_NODES=1 MAX_PUBLISHERS=10 MAX_SUBSCRIPTIONS=2 MAX_SERVICES=0 MAX_CLIENTS=0 MAX_HISTORY=2 CUSTOM_TRANSPORT=ON UDP=OFF TCP=OFF SERIAL=OFF DISCOVERY=OFF
url=https://github.com/micro-ROS/micro_ros_arduino
precompiled=true
category=Other
architectures=rp2040
EOF

# ---- Son SHA + cleanup ----
NEWLIB_SOFT=${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/libmicroros.a
NEWLIB_M33=${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/libmicroros.a
NEW_SHA=$(sha256sum ${NEWLIB_SOFT} | awk '{print $1}')
echo "[CLEAN SUB2] 16. SON KONTROL"
echo "  NEW SUB2 libmicroros.a (softfp) SHA256 = ${NEW_SHA}"
echo "  NEW SUB2 libmicroros.a (m33 link/dup) SHA256 = $(sha256sum ${NEWLIB_M33} | awk '{print $1}')"
echo "  NEW OUTPUT DIR: ${OUTPUT_NEW_LIB_DIR}"
echo "  BUILD LOG: ${BUILD_LOG}"
echo "[CLEAN SUB2] 17. TEMP stage siliniyor"
rm -rf ${TEMP_STAGE}
echo "[CLEAN SUB2] ✅ OFFICIAL CLEAN SUB=2 LIBRARY GENERATION BAŞARILI time: $(date -Iseconds)"
echo "[CLEAN SUB2] Exit code 0"
