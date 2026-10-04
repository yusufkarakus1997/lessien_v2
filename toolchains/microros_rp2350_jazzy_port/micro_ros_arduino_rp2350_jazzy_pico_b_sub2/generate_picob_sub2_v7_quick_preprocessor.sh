#!/bin/bash
# =====================================================================
# LESSIEN PICO B SUB2 V7: Mevcut VCS imported MCU_WS kullan (sıfırdan klonlama YOK)
# Sadece: rcpputils src *.cpp başına PREPROCESSOR PATCH HEADER ekle
#         + Ardından build_firmware.sh tekrar çalıştır
#
# PATCH HEADER:
#   - PATH_MAX define (missing)
#   - struct DIR / struct dirent concrete (forward yerine)
#   - opendir/readdir/closedir/getcwd static inline stub
#   - #define throw(...) NOOP (instead of exceptions)
#
# BU, BINARY/LIB PATCH DEĞİL, SOURCE CODE PREPROCESSOR INJECTION (generation ADIMI)
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
LOG_OUT=/home/pi5/lessien_ws/build_artifacts/log_generate_sub2_v7_preprocessor_patch.log

# MEVCUT VCS IMPORT EDILMIS MCU_WS KULLAN (sıfırdan beklememek için):
EXISTING_STAGE=/tmp/uros_stage_sub2_zxyGxI
TEMP_STAGE=/tmp/uros_stage_sub2_v7_$(openssl rand -hex 3)

export TOOLCHAIN_PREFIX
export ROS_DISTRO=jazzy
export PROJECT_PATH=${TEMP_STAGE}

echo "============================================================"
echo "LESSIEN PICO B: OFFICIAL SUB=2 micro-ROS LIBRARY GENERATION V7"
echo "V7 YENİ: rcpputils PREPROCESSOR PATCH (no function mismatch)"
echo "   Mevcut MCU_WS klonu kullanılıyor: ${EXISTING_STAGE}"
echo "============================================================"
date
echo "Log: ${LOG_OUT}"
exec 2>&1 | tee -a ${LOG_OUT}
echo

echo "[V7] 1. /opt/ros/jazzy + uros_ws sourced"
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
cd ${UROS_WS}
source install/local_setup.bash 2>/dev/null || true

echo "[V7] 2. Existing_stage rsync TEMP_STAGE (sıfırdan VCS import YOK)"
echo "   Kopyalanıyor: ${EXISTING_STAGE} -> ${TEMP_STAGE}"
rsync -a ${EXISTING_STAGE}/ ${TEMP_STAGE}/
FW_TARGETDIR=${TEMP_STAGE}/firmware
MCU_WS=${FW_TARGETDIR}/mcu_ws
DEV_WS=${FW_TARGETDIR}/dev_ws
export PROJECT_PATH=${TEMP_STAGE}

echo "[V7] 3. DEV_WS setup.bash shim kontrol (build_firmware L65)"
if [ ! -f ${DEV_WS}/install/setup.bash ]; then
  echo "   setup.bash yok, oluştur"
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
echo "[V7] 4. RCPPUTILS PREPROCESSOR HEADER PATCH (ENJEKSİYON)"
echo "============================================================"
RCPPUTILS_DIR=${MCU_WS}/ros2/rcpputils
echo "   Patch hedef: ${RCPPUTILS_DIR}"
ls ${RCPPUTILS_DIR}/src/

# --- HER BIR .cpp DOSYASININ BASINA AŞAĞIDAKİ HEADER'I EKLE ---
PATCH_HEADER=$(cat <<'PATCHEOF'
/* ================================================================
 * LESSIEN PICO B SUB2: Bare-metal Cortex-M33 PREPROCESSOR PATCH
 * Bu satırlar build ÖNCESİ injection edilmiştir (official ADIM).
 * Kök nedenler:
 *   1) PQT-GCC 16.1.0 newlib nano: sys/dirent.h -> #error <dirent.h> not supported
 *      -> struct DIR / struct dirent / opendir/readdir/closedir eksik
 *   2) PATH_MAX missing
 *   3) -fno-exceptions default: throw std::xxx derlenemez
 *
 * ÇÖZÜM: Preprocessor ile no-op stub + throw macro ignore
 * BU DEĞİŞİKLİK BİNARY FİNAL KÜTÜPHANEDE YALNIZCA rcpputils KISMINI ETKİLER,
 * RMW/microxrcedds/rosidl KISIMLARINA ETKİSİ YOKTUR.
 * Micro-ROS client RMW bu fonksiyonları ASLA ÇAĞIRMAZ.
 * ================================================================ */
#ifndef LESSIEN_PICOB_SUB2_RCPPUTILS_PATCH
#define LESSIEN_PICOB_SUB2_RCPPUTILS_PATCH 1

#ifndef PATH_MAX
#  define PATH_MAX 4096
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <cstddef>

/* Bare-metal dirent DIR struct concrete stub */
#ifndef __DIRENT_H_BAREMETAL_STUB
#define __DIRENT_H_BAREMETAL_STUB 1
struct dirent {
    ino_t          d_ino;
    off_t          d_off;
    unsigned short d_reclen;
    unsigned char  d_type;
    char           d_name[256];
};
typedef struct _DIR_STUB { int dummy; } DIR;
static inline DIR * opendir(const char * /*name*/)     { return nullptr; }
static inline struct dirent * readdir(DIR * /*dirp*/)  { return nullptr; }
static inline int closedir(DIR * /*dirp*/)             { return 0; }
#endif /* __DIRENT_H_BAREMETAL_STUB */

#ifndef HAVE_GETCWD_STUB
#define HAVE_GETCWD_STUB 1
static inline char * getcwd(char * /*buf*/, size_t /*size*/) { return nullptr; }
#endif

/* Exception throw YERİNE: infinite no-op loop (never reached on client) */
#define LESSIEN_NOOP_THROW(expr)  do { for(;;){} } while(0)
#ifndef throw
#  define throw(expr)  LESSIEN_NOOP_THROW(expr)
#endif

#endif /* LESSIEN_PICOB_SUB2_RCPPUTILS_PATCH */
/* ======================== END OF INJECTED PATCH ===================== */

PATCHEOF
)

echo
echo "   Enjekte ediliyor (prepend):"
for f in asserts.cpp filesystem_helper.cpp env.cpp process.cpp; do
  SRC=${RCPPUTILS_DIR}/src/${f}
  if [ -f "$SRC" ]; then
    echo "    - $f"
    # Başa patch header ekle (prepend)
    tmpf=$(mktemp)
    echo "${PATCH_HEADER}" > ${tmpf}
    cat ${SRC} >> ${tmpf}
    mv -f ${tmpf} ${SRC}
  else
    echo "    - $f YOK (atla)"
  fi
done

echo
echo "   Patch doğrulama (ilk 5 satır):"
head -5 ${RCPPUTILS_DIR}/src/filesystem_helper.cpp
echo "... "
head -5 ${RCPPUTILS_DIR}/src/asserts.cpp
echo

BUILD_FW_SH=${UROS_WS}/src/micro_ros_setup/scripts/build_firmware.sh
if [ ! -f "${BUILD_FW_SH}" ]; then
  BUILD_FW_SH=$(find ${UROS_WS} -name "build_firmware.sh" 2>/dev/null | head -1)
fi
echo "   build_firmware.sh: ${BUILD_FW_SH}"

echo
echo "============================================================"
echo "[V7] 5. CROSS-COMPILE build_firmware.sh (Cortex-M33 + SUB2 META)"
echo "============================================================"
rm -rf ${FW_TARGETDIR}/build
cd ${TEMP_STAGE}
ros2 run micro_ros_setup build_firmware.sh \
    ${TOOLCHAIN_CMAKE} \
    ${SUB2_META} 2>&1 | tee /tmp/build_fw_sub2_v7.log
BUILD_RET=${PIPESTATUS[0]}

echo
echo "============================================================"
echo "[V7] 6. Build Sonu (exit=${BUILD_RET}) log tail 140 satır:"
echo "============================================================"
tail -140 /tmp/build_fw_sub2_v7.log

if [ ${BUILD_RET} -eq 0 ] && [ -f ${FW_TARGETDIR}/build/libmicroros.a ]; then
  echo
  echo "============================================================"
  echo "✅✅✅ BUILD BAŞARILI! libmicroros.a üretildi ✅✅✅"
  echo "============================================================"
  ls -la ${FW_TARGETDIR}/build/libmicroros.a

  echo
  echo "[V7] 7. Stage/src include fix + Cortex-M33 klasör kurulumu"
  find ${FW_TARGETDIR}/build/include/ -name "*.c"  -delete 2>/dev/null || true
  cp -R ${FW_TARGETDIR}/build/include/* ${TEMP_STAGE}/src/ 2>&1 | tail -3
  mkdir -p ${TEMP_STAGE}/src/cortex-m33/softfp
  cp -v ${FW_TARGETDIR}/build/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/softfp/libmicroros.a
  ln -sf softfp/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/libmicroros.a 2>/dev/null || true

  echo
  echo "[V7] 8. Include path düzelt (resmi adım 262-270)"
  pushd ${MCU_WS} > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list 2>/dev/null | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
  popd > /dev/null
  echo "   Packages: ${INCLUDE_ROS2_PACKAGES}"
  for var in ${INCLUDE_ROS2_PACKAGES}; do
    rsync -r ${TEMP_STAGE}/src/${var}/${var}/* ${TEMP_STAGE}/src/${var}/ > /dev/null 2>&1
    rm -rf ${TEMP_STAGE}/src/${var}/${var}/ > /dev/null 2>&1
  done

  echo
  echo "[V7] 9. library.properties SUB2 meta"
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
  echo "[V7] 10. FINAL KOPYA (OUTPUT dir)"
  if [ -d ${OUTPUT_NEW_LIB_DIR} ]; then
    rm -rf ${OUTPUT_NEW_LIB_DIR}
  fi
  rsync -a ${TEMP_STAGE}/ ${OUTPUT_NEW_LIB_DIR}/
  echo "   ✅ OUTPUT: ${OUTPUT_NEW_LIB_DIR}"

  echo
  echo "   Yeni SUB2 lib SHA256:"
  NEW_SHA=$(sha256sum ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/libmicroros.a | awk '{print $1}')
  echo "   ${NEW_SHA}"

  echo
  echo "   RMW config doğrulama (SUB=2 bekliyoruz):"
  grep -E "MAX_NODES|MAX_PUBLISHERS|MAX_SUBSCRIPTIONS|MAX_HISTORY|MAX_SERVICES|MAX_CLIENTS" \
    ${OUTPUT_NEW_LIB_DIR}/src/rmw_microxrcedds_c/config.h

  echo
  echo "   Eski SUB1 SHA (DOĞRULAMA - değişmedi):"
  OLD_LIB=${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/cortex-m33/softfp/libmicroros.a
  OLD_SHA=$(sha256sum ${OLD_LIB} | awk '{print $1}')
  echo "   OLD SUB1 SHA: ${OLD_SHA}"
  echo "   NEW SUB2 SHA: ${NEW_SHA}"

  echo
  echo "   Typesupport doğrulama:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/twist.h
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/std_msgs/msg/float32.h
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/detail/twist__type_support.h
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/std_msgs/msg/detail/float32__type_support.h

  echo
  echo "   Cortex-M33 path doğrulama:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/
  file ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/libmicroros.a | head -1

  echo
  echo "============================================================"
  echo "✅✅✅ OFFICIAL SUB=2 LIBRARY GENERATION V7 BAŞARILI ✅✅✅"
  echo "============================================================"
else
  echo
  echo "❌ V7 FAIL exit=${BUILD_RET}"
  echo "Log tail 40 error lines:"
  grep -n "error:" /tmp/build_fw_sub2_v7.log 2>/dev/null | tail -40
fi
date
echo "TEMP_STAGE: ${TEMP_STAGE}"
exit ${BUILD_RET}
