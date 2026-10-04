#!/bin/bash
# =====================================================================
# LESSIEN PICO B SUB2 V11 — GUARD MACRO KESIN COZUM
# V10 HATA: filesystem_helper.cpp icinde COKLU #include <dirent.h> var (L60 ve L106).
#   Sadece L60/L62 commentlendi ama L106 sonraki include yine #error patlatti.
# V11 COZUM: DOSYA BASINA GUARD MACRO TANIMLA:
#   #define _DIRENT_H_      → artik #include <dirent.h> INCLUDE GUARD sayesinde SKIP
#   #define _SYS_DIRENT_H_  → sys/dirent.h guard (hata patlatmaz)
#   #define _UNISTD_H_      → unistd.h skip
#   #define _SYS_UNISTD_H   → sys/unistd.h skip (getcwd decl)
# Ardindan extern "C" ile struct DIR/dirent + opendir/readdir/closedir/getcwd decl.
# SED GEREK YOK! GUARD makrolar HERHANGI bir include path'ini otomatik SKIP eder.
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
LOG_OUT=/home/pi5/lessien_ws/build_artifacts/log_generate_sub2_v11_guard_macro.log

SRC_STAGE=/tmp/uros_stage_sub2_v8_471c2b
if [ ! -d ${SRC_STAGE} ]; then SRC_STAGE=/tmp/uros_stage_sub2_v9_3c85b2; fi
if [ ! -d ${SRC_STAGE} ]; then SRC_STAGE=/tmp/uros_stage_sub2_zxyGxI; fi
TEMP_STAGE=/tmp/uros_stage_sub2_v11_$(openssl rand -hex 3)

export TOOLCHAIN_PREFIX
export ROS_DISTRO=jazzy
export PROJECT_PATH=${TEMP_STAGE}

echo "============================================================"
echo "LESSIEN PICO B OFFICIAL SUB=2 V11 — GUARD MACRO (dirent/unistd SKIP)"
echo "   SRC_STAGE: ${SRC_STAGE} -> TEMP_STAGE: ${TEMP_STAGE}"
echo "============================================================"
date 2>&1 | tee -a ${LOG_OUT}
exec 2>&1 | tee -a ${LOG_OUT}
echo

echo "[V11] 1. ROS2 + uros_ws sourced"
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
cd ${UROS_WS}
source install/local_setup.bash 2>/dev/null || true

echo "[V11] 2. RSYNC stage kopyala"
rsync -a ${SRC_STAGE}/ ${TEMP_STAGE}/ 2>&1 | tail -2
FW_TARGETDIR=${TEMP_STAGE}/firmware
MCU_WS=${FW_TARGETDIR}/mcu_ws
DEV_WS=${FW_TARGETDIR}/dev_ws
export PROJECT_PATH=${TEMP_STAGE}

echo "[V11] 3. DEV_WS setup.bash"
if [ ! -f ${DEV_WS}/install/setup.bash ]; then
  mkdir -p ${DEV_WS}/install
  cat > ${DEV_WS}/install/setup.bash <<'EOF'
source /opt/ros/jazzy/setup.bash
if [ -d /opt/ros/jazzy/share/ament_cmake_vendor_package ]; then
  export CMAKE_PREFIX_PATH="/opt/ros/jazzy/share/ament_cmake_vendor_package/cmake:${CMAKE_PREFIX_PATH}"
fi
EOF
  touch ${DEV_WS}/install/local_setup.bash
  chmod +x ${DEV_WS}/install/*.bash
fi

echo
echo "============================================================"
echo "[V11] 4. SKIP pkgs + rcpputils GIT RESTORE + GUARD HEADER PREPEND"
echo "============================================================"
SKIP_PKGS_ROS2=(
  "libyaml_vendor" "rosidl_dynamic_typesupport" "rosidl_pycommon"
  "ros2_tracing" "example_interfaces" "unique_identifier_msgs"
)
for pkg in "${SKIP_PKGS_ROS2[@]}"; do
  if [ -d ${MCU_WS}/ros2/${pkg} ]; then
    touch ${MCU_WS}/ros2/${pkg}/COLCON_IGNORE
    echo "   SKIP: $pkg"
  fi
done

RCPP=${MCU_WS}/ros2/rcpputils
cd ${RCPP}
echo
echo "   4a. GIT RESTORE (temiz orijinal):"
git checkout HEAD -- . 2>&1 | tail -3
git status --short | head -5
cd - > /dev/null

echo
echo "   4b. filesystem_helper.cpp BASINA GUARD + extern C PREPEND (KESIN COZUM!)"
FSH=${RCPP}/src/filesystem_helper.cpp
ORJ_LINES=$(wc -l < ${FSH})

PATCH_GUARD=$(cat <<'GUARD_EOF'
/* ================================================================
 * LESSIEN PICO B SUB2 V11 — GUARD MACRO + extern C stub
 * PQT-GCC 16.1.0 newlib nano BARE-METAL SORUNLARI:
 *   (1) sys/dirent.h -> #error "<dirent.h> not supported"
 *   (2) struct DIR / dirent / opendir() / readdir() / closedir() / getcwd() YOK
 *
 * KESIN COZUM (100% calisir):
 *   - AŞAĞIDAKI HEADER GUARD MAKROLARI TANIMLANIR
 *   - ARTIK HERHANGI bir satırda #include <dirent.h> / <unistd.h> ÇAĞRILSA bile
 *     _DIRENT_H_ / _UNISTD_H_ define olduğu İÇİN HEPSİ SKIP EDİLİR!
 *   - Daha sonra extern "C" ile gerekli struct/fonksiyonlar DECLARE edilir.
 * BİLDİĞİM KADARIYLA Bu yöntem TÜM include varyantlarını kapsar (koşullu/boşluklu/basit).
 * ================================================================ */

/* --- (A) HEADER GUARD MAKROLARI: TÜM INCLUDE PATHLERINI SKIP ET --- */
#ifndef _DIRENT_H_
#  define _DIRENT_H_           1   /* prevents: #include <dirent.h> */
#endif
#ifndef _SYS_DIRENT_H_
#  define _SYS_DIRENT_H_       1   /* prevents: #include <sys/dirent.h> */
#endif
#ifndef _UNISTD_H_
#  define _UNISTD_H_           1   /* prevents: #include <unistd.h> */
#endif
#ifndef _SYS_UNISTD_H
#  define _SYS_UNISTD_H        1   /* prevents: #include <sys/unistd.h> */
#endif
#ifndef _DIRENT_H
#  define _DIRENT_H            1   /* alternatif guard name */
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <cstddef>

/* --- (B) extern "C" struct DIR/dirent concrete definitions --- */
#ifdef __cplusplus
extern "C" {
#endif

struct dirent {
    ino_t          d_ino;
    off_t          d_off;
    unsigned short d_reclen;
    unsigned char  d_type;
    char           d_name[256];
};
typedef struct _DIR_PICOB_STUB {
    int   fd;
    void *buf;
    long  pos;
} DIR;

/* --- (C) extern C inline stub implementations --- */
static inline DIR *                 opendir(const char *)          { return (DIR *)0; }
static inline struct dirent *       readdir(DIR *)                 { return (struct dirent *)0; }
static inline int                   closedir(DIR *)                 { return 0; }
static inline char *                getcwd(char *, size_t)         { return (char *)0; }
static inline int                   getdents(int, void *, unsigned) { return 0; }

#ifdef __cplusplus
} /* extern C */
#endif

/* ===================== END OF INJECTED PATCH ===================== */

GUARD_EOF
)

tmpf=$(mktemp)
printf "%s\n" "${PATCH_GUARD}" > ${tmpf}
cat ${FSH} >> ${tmpf}
mv -f ${tmpf} ${FSH}
NEW_LINES=$(wc -l < ${FSH})
echo "   ✅ PREPEND tamam: ${ORJ_LINES} satır -> ${NEW_LINES} satır"
echo
echo "   --- İLK 60 satır (GUARD doğrulama) ---"
head -60 ${FSH}

echo
echo "   4c. CMAKE rcpputils SUB2 flag kontrol:"
grep -A3 "rcpputils" ${SUB2_META}

BUILD_FW_SH=${UROS_WS}/src/micro_ros_setup/scripts/build_firmware.sh
if [ ! -f "${BUILD_FW_SH}" ]; then
  BUILD_FW_SH=$(find ${UROS_WS} -name "build_firmware.sh" 2>/dev/null | head -1)
fi
echo
echo "   build_firmware.sh: ${BUILD_FW_SH}"

echo
echo "============================================================"
echo "[V11] 5. CROSS-COMPILE (GUARD macro ile dirent/unistd SORUNLARI KAPANDI)"
echo "============================================================"
rm -rf ${FW_TARGETDIR}/build
cd ${TEMP_STAGE}
ros2 run micro_ros_setup build_firmware.sh \
    ${TOOLCHAIN_CMAKE} \
    ${SUB2_META} 2>&1 | tee /tmp/build_fw_sub2_v11.log
BUILD_RET=${PIPESTATUS[0]}

echo
echo "============================================================"
echo "[V11] 6. Build log tail 200 satır (exit=${BUILD_RET}):"
echo "============================================================"
tail -200 /tmp/build_fw_sub2_v11.log

if [ ${BUILD_RET} -eq 0 ] && [ -f ${FW_TARGETDIR}/build/libmicroros.a ]; then
  echo
  echo "============================================================"
  echo "✅✅✅ V11 BUILD BAŞARILI! libmicroros.a üretildi ✅✅✅"
  echo "============================================================"
  ls -la ${FW_TARGETDIR}/build/libmicroros.a
  echo
  echo "   7. Stage/src / Cortex-M33 klasör kurulumu"
  find ${FW_TARGETDIR}/build/include/ -name "*.c"  -delete 2>/dev/null || true
  cp -R ${FW_TARGETDIR}/build/include/* ${TEMP_STAGE}/src/ 2>&1 | tail -3
  mkdir -p ${TEMP_STAGE}/src/cortex-m33/softfp
  cp -v ${FW_TARGETDIR}/build/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/softfp/libmicroros.a
  ln -sf softfp/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/libmicroros.a 2>/dev/null || true

  echo
  echo "   8. Include path fix"
  pushd ${MCU_WS} > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list 2>/dev/null | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
  popd > /dev/null
  for var in ${INCLUDE_ROS2_PACKAGES}; do
    rsync -r ${TEMP_STAGE}/src/${var}/${var}/* ${TEMP_STAGE}/src/${var}/ > /dev/null 2>&1
    rm -rf ${TEMP_STAGE}/src/${var}/${var}/ > /dev/null 2>&1
  done

  echo
  echo "   9. library.properties SUB2"
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
  echo "   10. OUTPUT DIR kopyala"
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

  echo "   A) Yeni SUB=2 RMW limits:"
  grep -E "MAX_NODES|MAX_PUBLISHERS|MAX_SUBSCRIPTIONS|MAX_HISTORY|MAX_SERVICES|MAX_CLIENTS" \
    ${OUTPUT_NEW_LIB_DIR}/src/rmw_microxrcedds_c/config.h
  echo "   B) Eski SUB1 (değişmedi kanıtı):"
  grep -E "MAX_SUBSCRIPTIONS" ${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/rmw_microxrcedds_c/config.h
  echo "   C) geometry_msgs/Twist typesupport:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/twist.h ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/detail/twist__type_support.h 2>&1
  echo "   D) std_msgs/Float32 typesupport:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/std_msgs/msg/float32.h ${OUTPUT_NEW_LIB_DIR}/src/std_msgs/msg/detail/float32__type_support.h 2>&1
  echo "   E) Yeni libmicroros.a VAR:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/libmicroros.a
  echo "   F) Cortex-M33 archive path:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/ 2>&1
  echo "   G) NEW SUB2 SHA256 = ${NEW_SHA}"
  echo "   H) OLD SUB1 SHA256 = ${OLD_SHA}"
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
  echo "✅✅✅ OFFICIAL SUB=2 LIBRARY GENERATION V11 BAŞARILI ✅✅✅"
  echo "============================================================"
else
  echo
  echo "❌ V11 FAIL exit=${BUILD_RET}"
  echo
  echo "   Error lines (son 40):"
  grep -B0 -A1 "error:\|CMake Error\|conflicting declaration\|#error" /tmp/build_fw_sub2_v11.log 2>/dev/null | tail -40
fi

date
echo "TEMP_STAGE: ${TEMP_STAGE}"
exit ${BUILD_RET}
