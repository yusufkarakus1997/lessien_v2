#!/bin/bash
# =====================================================================
# LESSIEN PICO B SUB2 V10: KESIN COZUM
# V9 HATA KOKU: filesystem_helper.cpp -> #include <dirent.h> STUB'tan SONRA geliyor:
#   1) sys/dirent.h #error "<dirent.h> not supported" patlar
#   2) dirent.h C linkage: opendir/readdir/closedir benim static inline (C++ linkage) ILE CAKISIR
#   3) unistd.h getcwd C linkage benim C++ getcwd ILE CAKISIR
#
# V10 COZUM (ADIM ADIM KESIN):
#  1. rcpptutils GIT RESTORE (temiz orijinal)
#  2. filesystem_helper.cpp icindeki:
#       #include <dirent.h>  -> //#include <dirent.h>  (sed COMMENT OUT)
#       #include <unistd.h>  -> //#include <unistd.h>
#  3. Ardından dosya BASINA extern "C" ile struct DIR/dirent + opendir/readdir/closedir/getcwd DECLARE
#     -> C++ dosyasinda extern "C" ile yazinca C linkage olur, C++ STL derlenirken sorun olmaz
#  4. -fexceptions + PATH_MAX CMAKE_CXX_FLAGS AKTIF (colcon.meta V9'daki gibi)
#  5. Skip pkgs COLCON_IGNORE mevcut
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
LOG_OUT=/home/pi5/lessien_ws/build_artifacts/log_generate_sub2_v10_extern_c_sed.log

SRC_STAGE=/tmp/uros_stage_sub2_v8_471c2b
if [ ! -d ${SRC_STAGE} ]; then
  SRC_STAGE=/tmp/uros_stage_sub2_v9_3c85b2
fi
if [ ! -d ${SRC_STAGE} ]; then
  SRC_STAGE=/tmp/uros_stage_sub2_zxyGxI
fi
TEMP_STAGE=/tmp/uros_stage_sub2_v10_$(openssl rand -hex 3)

export TOOLCHAIN_PREFIX
export ROS_DISTRO=jazzy
export PROJECT_PATH=${TEMP_STAGE}

echo "============================================================"
echo "LESSIEN PICO B OFFICIAL SUB=2 V10 — extern C + SED comment dirent/unistd"
echo "   SRC_STAGE: ${SRC_STAGE} -> TEMP_STAGE: ${TEMP_STAGE}"
echo "============================================================"
date 2>&1 | tee -a ${LOG_OUT}
exec 2>&1 | tee -a ${LOG_OUT}
echo

echo "[V10] 1. ROS2 + uros_ws sourced"
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
cd ${UROS_WS}
source install/local_setup.bash 2>/dev/null || true

echo "[V10] 2. RSYNC"
rsync -a ${SRC_STAGE}/ ${TEMP_STAGE}/ 2>&1 | tail -2
FW_TARGETDIR=${TEMP_STAGE}/firmware
MCU_WS=${FW_TARGETDIR}/mcu_ws
DEV_WS=${FW_TARGETDIR}/dev_ws
export PROJECT_PATH=${TEMP_STAGE}

echo "[V10] 3. DEV_WS setup.bash"
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
echo "[V10] 4. COLCON_IGNORE + rcpputils GIT RESTORE + SED + extern C PREPEND"
echo "============================================================"
SKIP_PKGS_ROS2=(
  "libyaml_vendor" "rosidl_dynamic_typesupport" "rosidl_pycommon"
  "ros2_tracing" "example_interfaces" "unique_identifier_msgs"
)
for pkg in "${SKIP_PKGS_ROS2[@]}"; do
  if [ -d ${MCU_WS}/ros2/${pkg} ]; then
    touch ${MCU_WS}/ros2/${pkg}/COLCON_IGNORE
    echo "   SKIP: ${pkg}"
  fi
done
echo

RCPP=${MCU_WS}/ros2/rcpputils
cd ${RCPP}
echo "   4a. GIT RESTORE rcpptutils (orijinal temiz dosyalar)"
git checkout HEAD -- . 2>&1 | tail -3
git status --short 2>&1 | head -5
cd - > /dev/null

echo
echo "   4b. SED: COMMENT OUT #include <dirent.h> + <unistd.h> icinde filesystem_helper.cpp"
FSH=${RCPP}/src/filesystem_helper.cpp
ORJ_LINES=$(wc -l < ${FSH})
sed -i 's|^#include <dirent.h>|//#include <dirent.h>  // LESSIEN PICO B SUB2 V10: newlib nano dirent unsupported|g' ${FSH}
sed -i 's|^#include <unistd.h>|//#include <unistd.h>  // LESSIEN PICO B SUB2 V10: C linkage getcwd stub by prepended header|g' ${FSH}
echo "   SED uygulandı ($ORJ_LINES satir)"
grep -n "dirent.h\|unistd.h" ${FSH} | head -6

echo
echo "   4c. extern C PREPEND: struct DIR/dirent + opendir/readdir/closedir/getcwd (C linkage!)"
PATCH_HEAD=$(cat <<'EXTERN_C_EOF'
/* ================================================================
 * LESSIEN PICO B SUB2 V10: Bare-metal Cortex-M33 extern "C" stub
 * PQT-GCC 16.1.0 newlib nano:
 *    - sys/dirent.h -> #error "<dirent.h> not supported"
 *    - struct dirent / DIR / opendir/readdir/closedir / getcwd REAL IMPL YOK
 * COZUM:
 *    - #include <dirent.h> SED ile COMMENT OUT edildi
 *    - Asagida extern "C" ile struct + fonksiyonlar C linkage olarak DECLARE edildi
 *    - STATIC INLINE degil, ayni translation unit icinde implementation var
 * Bu header sadece bu C++ dosyasi icindir.
 * ================================================================ */
#ifndef LESSIEN_PICOB_V10_FS_EXTERNC
#define LESSIEN_PICOB_V10_FS_EXTERNC 1
#include <sys/types.h>
#include <sys/stat.h>
#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

/* --- concrete struct definitions (newlib nano bunlari define ETMEDI) --- */
struct dirent {
    ino_t          d_ino;
    off_t          d_off;
    unsigned short d_reclen;
    unsigned char  d_type;
    char           d_name[256];
};
typedef struct _DIR_BAREMETAL_STUB {
    int   _dummy_fd;
    void *_dummy_cookie;
} DIR;

/* --- stub implementations (same TU'da inline) --- */
static inline DIR * opendir(const char * /*name*/)           { return (DIR *)0; }
static inline struct dirent * readdir(DIR * /*dirp*/)       { return (struct dirent *)0; }
static inline int closedir(DIR * /*dirp*/)                   { return 0; }
static inline char * getcwd(char * /*buf*/, size_t /*size*/) { return (char *)0; }

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* LESSIEN_PICOB_V10_FS_EXTERNC */
/* ===================== END OF INJECTED PATCH ===================== */

EXTERN_C_EOF
)
tmpf=$(mktemp)
printf "%s\n" "${PATCH_HEAD}" > ${tmpf}
cat ${FSH} >> ${tmpf}
mv -f ${tmpf} ${FSH}
echo "   ✅ PREPEND tamamlandı. yeni satır: $(wc -l < ${FSH})"
echo "   --- ILK 35 SATIR (patch doğrulama) ---"
head -35 ${FSH}

echo
echo "   4d. CMAKE flag kontrol SUB2 meta:"
grep -A3 "rcpputils" ${SUB2_META}
echo

BUILD_FW_SH=${UROS_WS}/src/micro_ros_setup/scripts/build_firmware.sh
if [ ! -f "${BUILD_FW_SH}" ]; then
  BUILD_FW_SH=$(find ${UROS_WS} -name "build_firmware.sh" 2>/dev/null | head -1)
fi
echo "   build_firmware.sh: ${BUILD_FW_SH}"

echo
echo "============================================================"
echo "[V10] 5. CROSS-COMPILE build_firmware.sh (FINAL V10)"
echo "============================================================"
rm -rf ${FW_TARGETDIR}/build
cd ${TEMP_STAGE}
ros2 run micro_ros_setup build_firmware.sh \
    ${TOOLCHAIN_CMAKE} \
    ${SUB2_META} 2>&1 | tee /tmp/build_fw_sub2_v10.log
BUILD_RET=${PIPESTATUS[0]}

echo
echo "============================================================"
echo "[V10] 6. Build log tail 180 satır (exit=${BUILD_RET}):"
echo "============================================================"
tail -180 /tmp/build_fw_sub2_v10.log

if [ ${BUILD_RET} -eq 0 ] && [ -f ${FW_TARGETDIR}/build/libmicroros.a ]; then
  echo
  echo "============================================================"
  echo "✅✅✅ V10 BUILD BAŞARILI! libmicroros.a üretildi ✅✅✅"
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
  echo "   ===== 8 ADIM DOĞRULAMA (A-H) ====="
  NEW_SHA=$(sha256sum ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/libmicroros.a | awk '{print $1}')
  OLD_LIB=${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/cortex-m33/softfp/libmicroros.a
  OLD_SHA=$(sha256sum ${OLD_LIB} | awk '{print $1}')

  echo "   A) Yeni SUB=2 RMW limits:"
  grep -E "MAX_NODES|MAX_PUBLISHERS|MAX_SUBSCRIPTIONS|MAX_HISTORY|MAX_SERVICES|MAX_CLIENTS" \
    ${OUTPUT_NEW_LIB_DIR}/src/rmw_microxrcedds_c/config.h
  echo "   B) Eski SUB1 (değişmedi kanıtı):"
  grep -E "MAX_SUBSCRIPTIONS" ${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/rmw_microxrcedds_c/config.h
  echo "   C) geometry_msgs/Twist:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/twist.h ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/detail/twist__type_support.h 2>&1
  echo "   D) std_msgs/Float32:"
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
  echo "✅✅✅ OFFICIAL SUB=2 LIBRARY GENERATION V10 BAŞARILI ✅✅✅"
  echo "============================================================"
else
  echo
  echo "❌ V10 FAIL exit=${BUILD_RET}"
  echo
  echo "   Error lines (son 40):"
  grep -B0 -A1 "error:\|CMake Error\|conflicting declaration\|#error" /tmp/build_fw_sub2_v10.log 2>/dev/null | tail -40
fi

date
echo "TEMP_STAGE: ${TEMP_STAGE}"
exit ${BUILD_RET}
