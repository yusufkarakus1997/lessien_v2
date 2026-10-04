#!/bin/bash
# =====================================================================
# LESSIEN PICO B SUB2 V8: rcpputils GIT RESTORE + MINIMAL filesystem STUB
# ADIMLAR:
# 1. Var olan MCU_WS kullan (sıfırdan VCS import YOK ~5dk kazanç)
# 2. rcpputils GIT RESTORE (checkout HEAD -- *) -> V6/V7 patch temizle
# 3. colcon.meta: -DCMAKE_CXX_FLAGS="-fexceptions -DPATH_MAX=4096"  (THROW DERLENIR)
# 4. SADECE filesystem_helper.cpp BAŞINA: struct DIR/dirent/opendir/readdir/closedir/getcwd STUB prepend
#    (THROW MACRO YOK -> STL header çakışması ÖNLENDİ)
# 5. build_firmware.sh SUB2 meta ile cross-compile
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
LOG_OUT=/home/pi5/lessien_ws/build_artifacts/log_generate_sub2_v8_git_restore_minimal_stub.log

# Mevcut VCS imported MCU_WS (zaten klonlanmis, v8 icin rsync ile kopyala)
SRC_STAGE=/tmp/uros_stage_sub2_v7_0e5a71
if [ ! -d ${SRC_STAGE} ]; then
  SRC_STAGE=/tmp/uros_stage_sub2_zxyGxI
fi
TEMP_STAGE=/tmp/uros_stage_sub2_v8_$(openssl rand -hex 3)

export TOOLCHAIN_PREFIX
export ROS_DISTRO=jazzy
export PROJECT_PATH=${TEMP_STAGE}

echo "============================================================"
echo "LESSIEN PICO B OFFICIAL SUB=2 V8 — GIT RESTORE + MINIMAL STUB"
echo "   SRC_STAGE (clone): ${SRC_STAGE}"
echo "   TEMP_STAGE (build): ${TEMP_STAGE}"
echo "============================================================"
date 2>&1 | tee -a ${LOG_OUT}
exec 2>&1 | tee -a ${LOG_OUT}
echo

echo "[V8] 1. ROS2 + uros_ws sourced"
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
cd ${UROS_WS}
source install/local_setup.bash 2>/dev/null || true

echo "[V8] 2. RSYNC src_stage -> temp_stage (SIFIRDAN VCS YOK)"
rsync -a ${SRC_STAGE}/ ${TEMP_STAGE}/ 2>&1 | tail -3
FW_TARGETDIR=${TEMP_STAGE}/firmware
MCU_WS=${FW_TARGETDIR}/mcu_ws
DEV_WS=${FW_TARGETDIR}/dev_ws
export PROJECT_PATH=${TEMP_STAGE}
echo "   MCU_WS packages count: $(ls ${MCU_WS}/ros2/ | wc -l)"

echo "[V8] 3. DEV_WS setup.bash shim"
if [ ! -f ${DEV_WS}/install/setup.bash ]; then
  mkdir -p ${DEV_WS}/install
  cat > ${DEV_WS}/install/setup.bash <<'EOF'
source /opt/ros/jazzy/setup.bash
EOF
  cat > ${DEV_WS}/install/local_setup.bash <<'EOF'
EOF
  chmod +x ${DEV_WS}/install/*.bash
fi

echo
echo "============================================================"
echo "[V8] 4. rcpputils GIT RESTORE (V6/V7 patch KALDIR) + MINIMAL STUB"
echo "============================================================"
RCPP=${MCU_WS}/ros2/rcpputils
cd ${RCPP}
echo "   Adim 4a: git checkout HEAD (tum degisiklikleri geri al)"
git checkout HEAD -- . 2>&1 | tail -5
git status --short 2>&1 | head -8
cd - > /dev/null

echo
echo "   Adim 4b: filesystem_helper.cpp BAŞINA SADECE DIR/dirent/opendir stub prepend (THROW YOK!)"
FSH=${RCPP}/src/filesystem_helper.cpp
if [ -f "$FSH" ]; then
  PATCH_FS=$(cat <<'PATCH_FS_EOF'
/* LESSIEN PICO B SUB2 V8: Bare-metal filesystem STUB injection
 * PQT-GCC 16.1.0 newlib nano: sys/dirent.h -> #error <dirent.h> not supported
 * -> DIR, struct dirent, opendir/readdir/closedir, getcwd STUB olarak tanimlanir.
 * THROW KALDIRILDI: CMAKE_CXX_FLAGS ile -fexceptions zaten aktif.
 * Bu baslik sadece filesystem_helper.cpp icindir, STL header etkilenmez.
 */
#ifndef LESSIEN_PICOB_SUB2_V8_FS_STUB
#define LESSIEN_PICOB_SUB2_V8_FS_STUB 1
#include <sys/types.h>
#include <sys/stat.h>
#include <cstddef>
struct dirent {
    ino_t          d_ino;
    off_t          d_off;
    unsigned short d_reclen;
    unsigned char  d_type;
    char           d_name[256];
};
typedef struct _DIRLESS { int unused; } DIR;
static inline DIR * opendir(const char *)      { return nullptr; }
static inline struct dirent * readdir(DIR *)   { return nullptr; }
static inline int closedir(DIR *)               { return 0; }
static inline char * getcwd(char *, size_t)    { return nullptr; }
#endif

PATCH_FS_EOF
)
  tmpf=$(mktemp)
  echo "${PATCH_FS}" > ${tmpf}
  cat ${FSH} >> ${tmpf}
  mv -f ${tmpf} ${FSH}
  echo "   ✅ filesystem_helper.cpp prepend OK. Ilk 15 satir:"
  head -15 ${FSH}
fi

echo
echo "[V8] 5. colcon.meta rcpputils CXX flag kontrol:"
grep -A3 "rcpputils" ${SUB2_META}

BUILD_FW_SH=${UROS_WS}/src/micro_ros_setup/scripts/build_firmware.sh
if [ ! -f "${BUILD_FW_SH}" ]; then
  BUILD_FW_SH=$(find ${UROS_WS} -name "build_firmware.sh" 2>/dev/null | head -1)
fi
echo
echo "   build_firmware.sh: ${BUILD_FW_SH}"

echo
echo "============================================================"
echo "[V8] 6. CROSS-COMPILE build_firmware.sh Cortex-M33 softfp SUB2 meta"
echo "============================================================"
rm -rf ${FW_TARGETDIR}/build
cd ${TEMP_STAGE}
ros2 run micro_ros_setup build_firmware.sh \
    ${TOOLCHAIN_CMAKE} \
    ${SUB2_META} 2>&1 | tee /tmp/build_fw_sub2_v8.log
BUILD_RET=${PIPESTATUS[0]}

echo
echo "============================================================"
echo "[V8] 7. Build Sonu log tail 140 satır (exit=${BUILD_RET}):"
echo "============================================================"
tail -140 /tmp/build_fw_sub2_v8.log

if [ ${BUILD_RET} -eq 0 ] && [ -f ${FW_TARGETDIR}/build/libmicroros.a ]; then
  echo
  echo "============================================================"
  echo "✅✅✅ V8 BUILD BAŞARILI! libmicroros.a üretildi ✅✅✅"
  echo "============================================================"
  ls -la ${FW_TARGETDIR}/build/libmicroros.a

  echo
  echo "[V8] 8. Stage/src + Cortex-M33 klasör kurulumu"
  find ${FW_TARGETDIR}/build/include/ -name "*.c"  -delete 2>/dev/null || true
  cp -R ${FW_TARGETDIR}/build/include/* ${TEMP_STAGE}/src/ 2>&1 | tail -3
  mkdir -p ${TEMP_STAGE}/src/cortex-m33/softfp
  cp -v ${FW_TARGETDIR}/build/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/softfp/libmicroros.a
  ln -sf softfp/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/libmicroros.a 2>/dev/null || true

  echo
  echo "[V8] 9. Include path fix"
  pushd ${MCU_WS} > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list 2>/dev/null | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
  popd > /dev/null
  for var in ${INCLUDE_ROS2_PACKAGES}; do
    rsync -r ${TEMP_STAGE}/src/${var}/${var}/* ${TEMP_STAGE}/src/${var}/ > /dev/null 2>&1
    rm -rf ${TEMP_STAGE}/src/${var}/${var}/ > /dev/null 2>&1
  done

  echo
  echo "[V8] 10. library.properties SUB2"
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
  echo "[V8] 11. OUTPUT dir kopyala"
  if [ -d ${OUTPUT_NEW_LIB_DIR} ]; then
    rm -rf ${OUTPUT_NEW_LIB_DIR}
  fi
  rsync -a ${TEMP_STAGE}/ ${OUTPUT_NEW_LIB_DIR}/
  echo "   ✅ ${OUTPUT_NEW_LIB_DIR}"

  echo
  echo "   ===== 8 ADIMLI DOĞRULAMA ====="
  NEW_SHA=$(sha256sum ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/libmicroros.a | awk '{print $1}')
  OLD_LIB=${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/cortex-m33/softfp/libmicroros.a
  OLD_SHA=$(sha256sum ${OLD_LIB} | awk '{print $1}')

  echo "   A) Yeni RMW MAX_SUBSCRIPTIONS:"
  grep "MAX_SUBSCRIPTIONS" ${OUTPUT_NEW_LIB_DIR}/src/rmw_microxrcedds_c/config.h
  echo "   B) Eski SUB1 (değişmedi):"
  grep "MAX_SUBSCRIPTIONS" ${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/rmw_microxrcedds_c/config.h
  echo "   C) geometry_msgs/Twist:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/twist.h ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/detail/twist__type_support.h 2>&1
  echo "   D) std_msgs/Float32:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/std_msgs/msg/float32.h ${OUTPUT_NEW_LIB_DIR}/src/std_msgs/msg/detail/float32__type_support.h 2>&1
  echo "   E) Yeni libmicroros.a VAR:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/libmicroros.a
  echo "   F) Cortex-M33 path:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/ 2>&1
  echo "   G) NEW SHA256 = ${NEW_SHA}"
  echo "   H) OLD SHA256 = ${OLD_SHA}"
  if [ "${OLD_SHA}" = "89532b7d462a38783eabea41ea0d25577827cbf2cc99408bc383b64f2b9aaa1c" ]; then
    echo "   H+) ✅ Eski SUB1 lib SHA SABİT (immutable kanıtlandı)"
  else
    echo "   H+) ❌ UYARI: Eski SUB1 lib DEĞİŞMİŞ!"
  fi
  echo
  echo "   library.properties SUB2 doğrulama:"
  cat ${OUTPUT_NEW_LIB_DIR}/library.properties | head -7
  echo
  echo "============================================================"
  echo "✅✅✅ OFFICIAL SUB=2 LIBRARY GENERATION V8 TAMAMLANDI ✅✅✅"
  echo "============================================================"
else
  echo
  echo "❌ V8 FAIL exit=${BUILD_RET}"
  echo
  echo "   rcpputils errors:"
  grep -B1 -A2 "error:" /tmp/build_fw_sub2_v8.log 2>/dev/null | grep -v "Finished\|Aborted\|packages had\|not processed\|Summary\|package" | tail -40
fi

date
echo "TEMP_STAGE: ${TEMP_STAGE}  (inceleme için)"
exit ${BUILD_RET}
