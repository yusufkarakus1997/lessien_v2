#!/bin/bash
# =====================================================================
# LESSIEN PICO B SUB2 V9: GEREKSIZ PAKETLERI COLCON_IGNORE + rcpputils build devam
# V8 ANALIZ:
#   - rcpputils: HATA YOKTU, sadece libyaml_vendor fail oldugu icin ABORT edildi
#   - libyaml_vendor: Cortex-M33 client TARAFINDA KULLANILMAZ (host-only yaml vendor)
#   - rosidl_dynamic_typesupport: client'ta kullanilmaz (dynamic typesupport)
#   - rosidl_pycommon: PYTHON, Cortex-M33'de Python YOK
#
# COZUM: Bu 3 + benzer host-only paketlere COLCON_IGNORE touch et, ardindan tekrar build.
# Mevcut MCU_WS tekrar kullan (sifirdan VCS import YOK).
# =====================================================================
set +e

UROS_WS=/home/pi5/lessien_ws/toolchains/uros_ws
STAGE=/home/pi5/lessien_ws/toolchains/pico_b_microros_stage
PORT=/home/pi5/lessien_ws/toolchains/microros_rp2350_jazzy_port
TOOLCHAIN_PREFIX=/home/pi5/.arduino15/packages/rp2040/tools/pqt-gcc/5.0.0-9576866/bin/arm-none-eabi-
TOOLCHAIN_CMAKE=${STAGE}/extras/library_generation/rp2350_arduino_pico_toolchain.cmake
SUB2_META=${STAGE}/extras/library_generation/colcon_rp2350_pico_b_sub2.meta
OUTPUT_NEW_LIB_DIR=${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b_sub2
NEW_LIBRARY_NAME=micro_ros_arduino_pico_b_sub2
NEW_LIBRARY_VERSION=2.0.8-jazzy-picob-sub2
LOG_OUT=/home/pi5/lessien_ws/build_artifacts/log_generate_sub2_v9_skip_pkgs.log

SRC_STAGE=/tmp/uros_stage_sub2_v8_471c2b
if [ ! -d ${SRC_STAGE} ]; then
  SRC_STAGE=/tmp/uros_stage_sub2_v7_0e5a71
fi
TEMP_STAGE=/tmp/uros_stage_sub2_v9_$(openssl rand -hex 3)

export TOOLCHAIN_PREFIX
export ROS_DISTRO=jazzy
export PROJECT_PATH=${TEMP_STAGE}

echo "============================================================"
echo "LESSIEN PICO B OFFICIAL SUB=2 V9 — SKIP HOST-ONLY PKGS + BUILD"
echo "   SRC_STAGE: ${SRC_STAGE} -> TEMP_STAGE: ${TEMP_STAGE}"
echo "============================================================"
date 2>&1 | tee -a ${LOG_OUT}
exec 2>&1 | tee -a ${LOG_OUT}
echo

echo "[V9] 1. ROS2 + uros_ws sourced"
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
cd ${UROS_WS}
source install/local_setup.bash 2>/dev/null || true

echo "[V9] 2. RSYNC src_stage -> temp_stage"
rsync -a ${SRC_STAGE}/ ${TEMP_STAGE}/ 2>&1 | tail -2
FW_TARGETDIR=${TEMP_STAGE}/firmware
MCU_WS=${FW_TARGETDIR}/mcu_ws
DEV_WS=${FW_TARGETDIR}/dev_ws
export PROJECT_PATH=${TEMP_STAGE}

echo "[V9] 3. DEV_WS setup.bash shim"
if [ ! -f ${DEV_WS}/install/setup.bash ]; then
  mkdir -p ${DEV_WS}/install
  cat > ${DEV_WS}/install/setup.bash <<'EOF'
source /opt/ros/jazzy/setup.bash
# Eksik ament_cmake_vendor_package icin prefix path genislet (varsa):
if [ -d /opt/ros/jazzy/share/ament_cmake_vendor_package ]; then
  export CMAKE_PREFIX_PATH="/opt/ros/jazzy/share/ament_cmake_vendor_package/cmake:${CMAKE_PREFIX_PATH}"
fi
EOF
  touch ${DEV_WS}/install/local_setup.bash
  chmod +x ${DEV_WS}/install/*.bash
fi

echo
echo "============================================================"
echo "[V9] 4. COLCON_IGNORE: Host-only / Client icin GEREKSIZ paketler SKIP"
echo "============================================================"
# Bu paketler Cortex-M33 micro-ROS CLIENT icin gereksiz:
SKIP_PKGS_ROS2=(
  "libyaml_vendor"
  "rosidl_dynamic_typesupport"
  "rosidl_pycommon"
  "ros2_tracing"
  "example_interfaces"
  "unique_identifier_msgs"
)
echo "   SKIP edilecek paketler (ros2/): ${SKIP_PKGS_ROS2[*]}"
for pkg in "${SKIP_PKGS_ROS2[@]}"; do
  if [ -d ${MCU_WS}/ros2/${pkg} ]; then
    touch ${MCU_WS}/ros2/${pkg}/COLCON_IGNORE
    echo "    + ${pkg} -> COLCON_IGNORE ✅"
  fi
done
echo
echo "   MCU_WS/ros2 packages after skip: $(ls ${MCU_WS}/ros2/ | wc -l)"
ls ${MCU_WS}/ros2/

echo
echo "   rcpputils patch kontrol:"
RCPP=${MCU_WS}/ros2/rcpputils
if grep -q "LESSIEN_PICOB_SUB2_V8_FS_STUB" ${RCPP}/src/filesystem_helper.cpp 2>/dev/null; then
  echo "    ✅ V8 filesystem stub MEVCUT"
else
  echo "    ⚠️  V8 stub YOK - yeniden uygula"
  PATCH_FS=$(cat <<'PATCH_FS_EOF'
/* LESSIEN PICO B SUB2 V8: Bare-metal filesystem STUB injection */
#ifndef LESSIEN_PICOB_SUB2_V8_FS_STUB
#define LESSIEN_PICOB_SUB2_V8_FS_STUB 1
#include <sys/types.h>
#include <sys/stat.h>
#include <cstddef>
struct dirent { ino_t d_ino; off_t d_off; unsigned short d_reclen; unsigned char d_type; char d_name[256]; };
typedef struct _DIRLESS { int unused; } DIR;
static inline DIR * opendir(const char *) { return nullptr; }
static inline struct dirent * readdir(DIR *) { return nullptr; }
static inline int closedir(DIR *) { return 0; }
static inline char * getcwd(char *, size_t) { return nullptr; }
#endif

PATCH_FS_EOF
)
  tmpf=$(mktemp)
  echo "${PATCH_FS}" > ${tmpf}
  cat ${RCPP}/src/filesystem_helper.cpp >> ${tmpf}
  mv -f ${tmpf} ${RCPP}/src/filesystem_helper.cpp
  echo "    ✅ Yeniden uygulandı"
fi

BUILD_FW_SH=${UROS_WS}/src/micro_ros_setup/scripts/build_firmware.sh
if [ ! -f "${BUILD_FW_SH}" ]; then
  BUILD_FW_SH=$(find ${UROS_WS} -name "build_firmware.sh" 2>/dev/null | head -1)
fi
echo
echo "   build_firmware.sh: ${BUILD_FW_SH}"

echo
echo "============================================================"
echo "[V9] 5. CROSS-COMPILE build_firmware.sh (SUB2 meta + SKIP pkgs)"
echo "============================================================"
rm -rf ${FW_TARGETDIR}/build
cd ${TEMP_STAGE}
ros2 run micro_ros_setup build_firmware.sh \
    ${TOOLCHAIN_CMAKE} \
    ${SUB2_META} 2>&1 | tee /tmp/build_fw_sub2_v9.log
BUILD_RET=${PIPESTATUS[0]}

echo
echo "============================================================"
echo "[V9] 6. Build log tail 160 satır (exit=${BUILD_RET}):"
echo "============================================================"
tail -160 /tmp/build_fw_sub2_v9.log

if [ ${BUILD_RET} -eq 0 ] && [ -f ${FW_TARGETDIR}/build/libmicroros.a ]; then
  echo
  echo "============================================================"
  echo "✅✅✅ V9 BUILD BAŞARILI! libmicroros.a üretildi ✅✅✅"
  echo "============================================================"
  ls -la ${FW_TARGETDIR}/build/libmicroros.a

  echo
  echo "[V9] 7. Stage/src + Cortex-M33 klasör kurulumu"
  find ${FW_TARGETDIR}/build/include/ -name "*.c"  -delete 2>/dev/null || true
  cp -R ${FW_TARGETDIR}/build/include/* ${TEMP_STAGE}/src/ 2>&1 | tail -3
  mkdir -p ${TEMP_STAGE}/src/cortex-m33/softfp
  cp -v ${FW_TARGETDIR}/build/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/softfp/libmicroros.a
  ln -sf softfp/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/libmicroros.a 2>/dev/null || true

  echo
  echo "[V9] 8. Include path fix (resmi adım)"
  pushd ${MCU_WS} > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list 2>/dev/null | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
  popd > /dev/null
  for var in ${INCLUDE_ROS2_PACKAGES}; do
    rsync -r ${TEMP_STAGE}/src/${var}/${var}/* ${TEMP_STAGE}/src/${var}/ > /dev/null 2>&1
    rm -rf ${TEMP_STAGE}/src/${var}/${var}/ > /dev/null 2>&1
  done

  echo
  echo "[V9] 9. library.properties SUB2"
  cat > ${TEMP_STAGE}/library.properties <<EOF
name=${NEW_LIBRARY_NAME}
version=${NEW_LIBRARY_VERSION}
author=micro-ROS team, Lessien Pico B SUB2 variant
maintainer=Lessien
sentence=Official micro-ROS for Arduino RP2350 Pico B - MAX_SUBSCRIPTIONS=2 variant
paragraph=RMW Limits: NODES=1, PUBLISHERS=10, MAX_SUBSCRIPTIONS=2, HISTORY=2, SERVICES=0, CLIENTS=0. CUSTOM_TRANSPORT=ON. Cortex-M33 softfp.
category=Communication
url=https://micro.ros.org
architectures=rp2040
includes=micro_ros_arduino.h
dot_a_linkage=true
precompiled=true
ldflags=-lm
EOF

  echo
  echo "[V9] 10. OUTPUT dir kopyala"
  if [ -d ${OUTPUT_NEW_LIB_DIR} ]; then
    rm -rf ${OUTPUT_NEW_LIB_DIR}
  fi
  rsync -a ${TEMP_STAGE}/ ${OUTPUT_NEW_LIB_DIR}/
  echo "   ✅ ${OUTPUT_NEW_LIB_DIR}"

  echo
  echo "   ===== 8 ADIMLI DOĞRULAMA (A-H) ====="
  NEW_SHA=$(sha256sum ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/libmicroros.a | awk '{print $1}')
  OLD_LIB=${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/cortex-m33/softfp/libmicroros.a
  OLD_SHA=$(sha256sum ${OLD_LIB} | awk '{print $1}')

  echo "   A) Yeni SUB=2 RMW config:"
  grep -E "MAX_NODES|MAX_PUBLISHERS|MAX_SUBSCRIPTIONS|MAX_HISTORY|MAX_SERVICES|MAX_CLIENTS" \
    ${OUTPUT_NEW_LIB_DIR}/src/rmw_microxrcedds_c/config.h
  echo "   B) Eski SUB1 (değişmedi kanıt):"
  grep -E "MAX_SUBSCRIPTIONS" ${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/rmw_microxrcedds_c/config.h
  echo "   C) geometry_msgs/Twist typesupport:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/twist.h ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/detail/twist__type_support.h 2>&1
  echo "   D) std_msgs/Float32 typesupport:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/std_msgs/msg/float32.h ${OUTPUT_NEW_LIB_DIR}/src/std_msgs/msg/detail/float32__type_support.h 2>&1
  echo "   E) Yeni libmicroros.a VAR:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/libmicroros.a
  echo "   F) Cortex-M33 ARCHIVE PATH (src/cortex-m33/softfp):"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/ 2>&1
  echo "   G) NEW SHA256 = ${NEW_SHA}"
  echo "   H) OLD SHA256 = ${OLD_SHA}"
  if [ "${OLD_SHA}" = "89532b7d462a38783eabea41ea0d25577827cbf2cc99408bc383b64f2b9aaa1c" ]; then
    echo "   H+) ✅ Eski SUB1 lib SHA SABİT — immutable KANITLANDI"
  else
    echo "   H+) ❌ UYARI: Eski SUB1 lib DEĞİŞMİŞ!"
  fi
  echo
  echo "   library.properties SUB2 doğrulama:"
  cat ${OUTPUT_NEW_LIB_DIR}/library.properties | head -7
  echo
  echo "============================================================"
  echo "✅✅✅ OFFICIAL SUB=2 LIBRARY GENERATION V9 BAŞARILI ✅✅✅"
  echo "============================================================"
else
  echo
  echo "❌ V9 FAIL exit=${BUILD_RET}"
  echo
  echo "   Error lines (son 40 hata):"
  grep -B0 -A1 "error:\|CMake Error" /tmp/build_fw_sub2_v9.log 2>/dev/null | tail -40
fi

date
echo "TEMP_STAGE: ${TEMP_STAGE}  (inceleme için)"
exit ${BUILD_RET}
