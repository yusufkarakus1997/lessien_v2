#!/bin/bash
# =====================================================================
# LESSIEN PICO B SUB2 V12 — GUARD MACRO + FULL POSIX STUBS
# V11 HATA: _UNISTD_H_ guard yapınca access()/mkdir()/rmdir() DECLARE EDİLMEDİ
# V12 ÇÖZÜM: Tüm kullanılan POSIX fonksiyonlarını GUARD sonrası EXTERN C ile DECLARE ET:
#   access(), mkdir(), rmdir(), unlink(), chdir(), rename(), symlink(), readlink()
#   errno + EEXIST + makrolar
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
LOG_OUT=/home/pi5/lessien_ws/build_artifacts/log_generate_sub2_v12_full_stubs.log

SRC_STAGE=/tmp/uros_stage_sub2_v8_471c2b
if [ ! -d ${SRC_STAGE} ]; then SRC_STAGE=/tmp/uros_stage_sub2_v9_3c85b2; fi
if [ ! -d ${SRC_STAGE} ]; then SRC_STAGE=/tmp/uros_stage_sub2_zxyGxI; fi
TEMP_STAGE=/tmp/uros_stage_sub2_v12_$(openssl rand -hex 3)

export TOOLCHAIN_PREFIX
export ROS_DISTRO=jazzy
export PROJECT_PATH=${TEMP_STAGE}

echo "============================================================"
echo "LESSIEN PICO B OFFICIAL SUB=2 V12 — GUARD + FULL POSIX STUB"
echo "   SRC_STAGE: ${SRC_STAGE} -> TEMP_STAGE: ${TEMP_STAGE}"
echo "============================================================"
date 2>&1 | tee -a ${LOG_OUT}
exec 2>&1 | tee -a ${LOG_OUT}
echo

echo "[V12] 1. ROS2 + uros_ws sourced"
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
cd ${UROS_WS}
source install/local_setup.bash 2>/dev/null || true

echo "[V12] 2. RSYNC stage kopyala"
rsync -a ${SRC_STAGE}/ ${TEMP_STAGE}/ 2>&1 | tail -2
FW_TARGETDIR=${TEMP_STAGE}/firmware
MCU_WS=${FW_TARGETDIR}/mcu_ws
DEV_WS=${FW_TARGETDIR}/dev_ws
export PROJECT_PATH=${TEMP_STAGE}

echo "[V12] 3. DEV_WS setup.bash"
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
echo "[V12] 4. SKIP pkgs + rcpputils GIT RESTORE + FULL GUARD+STUB"
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
echo "   4b. filesystem_helper.cpp BASINA GUARD + extern C FULL STUB"
FSH=${RCPP}/src/filesystem_helper.cpp
ORJ_LINES=$(wc -l < ${FSH})

PATCH_GUARD=$(cat <<'GUARD_EOF'
/* ================================================================
 * LESSIEN PICO B SUB2 V12 — GUARD MACRO + FULL POSIX STUB
 * KESIN COZUM:
 *   (1) GUARD makrolari ile <dirent.h>/<unistd.h>/<sys/dirent.h> HEPSINI SKIP ET
 *   (2) Ardindan EXTERN C ile KULLANILAN TUM POSIX FONKSIYONLARINI DECLARE ET
 * ================================================================ */

/* --- (A) HEADER GUARD MAKROLARI --- */
#ifndef _DIRENT_H_
#  define _DIRENT_H_           1
#endif
#ifndef _SYS_DIRENT_H_
#  define _SYS_DIRENT_H_       1
#endif
#ifndef _DIRENT_H
#  define _DIRENT_H            1
#endif
#ifndef _UNISTD_H_
#  define _UNISTD_H_           1
#endif
#ifndef _SYS_UNISTD_H
#  define _SYS_UNISTD_H        1
#endif
#ifndef _UNISTD_H
#  define _UNISTD_H            1
#endif
#ifndef _FCNTL_H_
#  define _FCNTL_H_            1
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <stddef.h>
#include <stdint.h>

/* --- (B) ERRNO + POSIX MAKROLARI --- */
#ifndef errno
extern int errno;
#endif
#ifndef EEXIST
#  define EEXIST      17
#endif
#ifndef ENOENT
#  define ENOENT       2
#endif
#ifndef EACCES
#  define EACCES      13
#endif
#ifndef EINVAL
#  define EINVAL      22
#endif
#ifndef ENOMEM
#  define ENOMEM      12
#endif
#ifndef ENOTDIR
#  define ENOTDIR     20
#endif
#ifndef R_OK
#  define R_OK        4
#endif
#ifndef W_OK
#  define W_OK        2
#endif
#ifndef X_OK
#  define X_OK        1
#endif
#ifndef F_OK
#  define F_OK        0
#endif
#ifndef S_IRWXU
#  define S_IRWXU   0700
#endif
#ifndef S_IRWXG
#  define S_IRWXG   0070
#endif
#ifndef S_IRWXO
#  define S_IRWXO   0007
#endif
#ifndef S_IRUSR
#  define S_IRUSR   0400
#endif
#ifndef S_IWUSR
#  define S_IWUSR   0200
#endif
#ifndef S_IXUSR
#  define S_IXUSR   0100
#endif
#ifndef S_IFMT
#  define S_IFMT   0170000
#endif
#ifndef S_IFDIR
#  define S_IFDIR  0040000
#endif
#ifndef S_IFREG
#  define S_IFREG  0100000
#endif
#ifndef S_ISDIR
#  define S_ISDIR(m) (((m) & S_IFMT) == S_IFDIR)
#endif
#ifndef S_ISREG
#  define S_ISREG(m) (((m) & S_IFMT) == S_IFREG)
#endif

/* --- (C) extern "C" struct DIR + dirent --- */
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

/* --- (D) FULL POSIX FUNCTION STUBS (KULLANILAN TUMU) --- */
static inline DIR *                 opendir(const char *)               { return (DIR *)0; }
static inline struct dirent *       readdir(DIR *)                      { return (struct dirent *)0; }
static inline int                   closedir(DIR *)                      { return 0; }
static inline char *                getcwd(char *, size_t)              { return (char *)0; }
static inline int                   getdents(int, void *, unsigned)     { return 0; }
static inline int                   access(const char *, int)           { return -1; }
static inline int                   mkdir(const char *, mode_t)         { return -1; }
static inline int                   rmdir(const char *)                 { return -1; }
static inline int                   unlink(const char *)                { return -1; }
static inline int                   chdir(const char *)                 { return -1; }
static inline int                   rename(const char *, const char *)  { return -1; }
static inline int                   symlink(const char *, const char *) { return -1; }
static inline int                   readlink(const char *, char *, size_t) { return -1; }
static inline int                   isatty(int)                         { return 0; }
static inline pid_t                 getpid(void)                        { return 1; }
static inline char *                getenv(const char *)                { return (char *)0; }
static inline int                   putenv(char *)                      { return -1; }
static inline int                   chmod(const char *, mode_t)         { return -1; }
static inline int                   stat(const char *p, struct stat *s) { (void)p; (void)s; return -1; }
static inline int                   lstat(const char *p, struct stat *s){ (void)p; (void)s; return -1; }

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
echo "   --- Patch sonrası L157 access() ve L446 mkdir() satırları ---"
grep -nE "return access|mkdir\(p_built" ${FSH}
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
echo "[V12] 5. CROSS-COMPILE (FULL STUB ile access/mkdir KAPANDI)"
echo "============================================================"
rm -rf ${FW_TARGETDIR}/build
cd ${TEMP_STAGE}
ros2 run micro_ros_setup build_firmware.sh \
    ${TOOLCHAIN_CMAKE} \
    ${SUB2_META} 2>&1 | tee /tmp/build_fw_sub2_v12.log
BUILD_RET=${PIPESTATUS[0]}

echo
echo "============================================================"
echo "[V12] 6. Build log tail 250 satır (exit=${BUILD_RET}):"
echo "============================================================"
tail -250 /tmp/build_fw_sub2_v12.log

if [ ${BUILD_RET} -eq 0 ] && [ -f ${FW_TARGETDIR}/build/libmicroros.a ]; then
  echo
  echo "============================================================"
  echo "✅✅✅ V12 BUILD BAŞARILI! libmicroros.a üretildi ✅✅✅"
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
  echo "✅✅✅ OFFICIAL SUB=2 LIBRARY GENERATION V12 BAŞARILI ✅✅✅"
  echo "============================================================"
else
  echo
  echo "❌ V12 FAIL exit=${BUILD_RET}"
  echo
  echo "   Error lines (son 50):"
  grep -B0 -A2 "error:\|CMake Error\|conflicting declaration\|#error\|was not declared\|undefined reference" /tmp/build_fw_sub2_v12.log 2>/dev/null | tail -50
fi

date
echo "TEMP_STAGE: ${TEMP_STAGE}"
exit ${BUILD_RET}
