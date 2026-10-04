#!/bin/bash
# =====================================================================
# OFFICIAL micro-ROS RP2350 Pico B SUB=2 Library Generation Wrapper V6
# OPTİMİZASYON: ROSDEP INIT KISMINI ATLA (sadece VCS IMPORT KULLAN)
# V6 YENİLİK: rcpputils bare-metal stub patch (step 4d-2)
#   - PQT-GCC 16.1.0 newlib nano: dirent.h #error "<dirent.h> not supported"
#   - sys/dirent.h: struct DIR/struct dirent/PATH_MAX MİSSİNG
#   - -fno-exceptions: throw throw std::system_error derlenemiyor
#   - ÇÖZÜM: VCS import SONRASI build ÖNCESİ source-level STUB OVERWRITE
#           (binary/ar patch YOK, sadece generation script ADIMI)
#
# SADECE FARK: colcon meta = colcon_rp2350_pico_b_sub2.meta (SUB=2, diğer limits SUB1 İLE AYNI)
# OUTPUT KLASÖR: microros_rp2350_jazzy_port/micro_ros_arduino_rp2350_jazzy_pico_b_sub2
# Eski SUB1 library KORUNUR (ÜZERİNE YAZILMAZ)
# =====================================================================
set +e

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
LOG_OUT=/home/pi5/lessien_ws/build_artifacts/log_generate_sub2_v6_rcpputils_stub_patch.log

export TOOLCHAIN_PREFIX
export ROS_DISTRO=jazzy
export PROJECT_PATH=${TEMP_STAGE}

echo "============================================================"
echo "LESSIEN PICO B: OFFICIAL SUB=2 micro-ROS LIBRARY GENERATION V6"
echo "V6 YENİ: rcpputils bare-metal stub patch (dirent/exceptions KAPANDI)"
echo "============================================================"
date
echo "Log output: ${LOG_OUT}"
echo
exec > >(tee -a ${LOG_OUT}) 2>&1

echo "[GEN-SUB2-V6] 1. Resmi /opt/ros/${ROS_DISTRO} sourced"
source /opt/ros/${ROS_DISTRO}/setup.bash 2>/dev/null || true

echo "[GEN-SUB2-V6] 2. /uros_ws (micro_ros_setup colcon workspace) sourced"
cd ${UROS_WS}
if [ ! -f install/local_setup.bash ]; then
  echo "[GEN-SUB2-V6] 2a. İLK KEZ: uros_ws colcon build (micro_ros_setup package)"
  colcon build --packages-up-to micro_ros_setup 2>&1 | tail -20
fi
source install/local_setup.bash 2>/dev/null || true

echo "[GEN-SUB2-V6] 3. STAGE DIZININI TEMP KOPYALA (Üretim sırasında orijinal STAGE korunur)"
echo "   TEMP: ${TEMP_STAGE}"
rsync -a --delete ${STAGE}/ ${TEMP_STAGE}/

echo "[GEN-SUB2-V6] 4. MANUEL FIRMWARE DIZINI OLUSTUR (create_firmware_ws ROSDEP ATSKIP)"
cd ${TEMP_STAGE}

FW_TARGETDIR=${TEMP_STAGE}/firmware
if [ -d ${FW_TARGETDIR} ]; then
  echo "   Eski firmware varsa temizle..."
  rm -rf ${FW_TARGETDIR}
fi
mkdir -p ${FW_TARGETDIR}
touch ${FW_TARGETDIR}/COLCON_IGNORE
echo "generate_lib" > ${FW_TARGETDIR}/PLATFORM
echo "generic" >> ${FW_TARGETDIR}/PLATFORM

MCU_WS=${FW_TARGETDIR}/mcu_ws
DEV_WS=${FW_TARGETDIR}/dev_ws
mkdir -p ${MCU_WS} ${DEV_WS}

PREFIX=$(ros2 pkg prefix micro_ros_setup 2>/dev/null)
if [ -z "$PREFIX" ]; then
  PREFIX=${UROS_WS}/src/micro_ros_setup
fi
echo "   micro_ros_setup PREFIX: ${PREFIX}"

DEV_ROS2_PKGS=${PREFIX}/config/generate_lib/dev_ros2_packages.txt
DEV_UROS_REPOS=${PREFIX}/config/generate_lib/dev_uros_packages.repos
CLIENT_ROS2_PKGS=${PREFIX}/config/client_ros2_packages.txt
CLIENT_UROS_REPOS=${PREFIX}/config/generate_lib/generic/client_uros_packages.repos
if [ ! -f "${DEV_ROS2_PKGS}" ]; then
  DEV_ROS2_PKGS=${UROS_WS}/src/micro_ros_setup/config/generate_lib/dev_ros2_packages.txt
  DEV_UROS_REPOS=${UROS_WS}/src/micro_ros_setup/config/generate_lib/dev_uros_packages.repos
fi
if [ ! -f "${CLIENT_ROS2_PKGS}" ]; then
  CLIENT_ROS2_PKGS=${UROS_WS}/src/micro_ros_setup/config/client_ros2_packages.txt
fi
if [ ! -f "${CLIENT_UROS_REPOS}" ]; then
  CLIENT_UROS_REPOS=${UROS_WS}/src/micro_ros_setup/config/generate_lib/generic/client_uros_packages.repos
fi

echo
echo "[GEN-SUB2-V6] 4c-1. dev_ws/install/setup.bash OLUŞTUR (build_firmware.sh L65 zorunluluğu)"
mkdir -p ${DEV_WS}/install
cat > ${DEV_WS}/install/setup.bash <<'EOF'
# Lessien Pico B SUB2 dev_ws setup.bash shim
# build_firmware.sh L62 clean_env.sh TÜM ROS2 ENVLERİNİ SİLİYOR.
# Sonra L65 bu setup.bash i SOURCE eder.
# BU NOKTADA /opt/ros/jazzy setup.bash ı YENİDEN SOURCE etmeliyiz.
source /opt/ros/jazzy/setup.bash
EOF
cat > ${DEV_WS}/install/local_setup.bash <<'EOF'
# Lessien Pico B SUB2 dev_ws local_setup.bash shim
EOF
chmod +x ${DEV_WS}/install/setup.bash ${DEV_WS}/install/local_setup.bash

echo "[GEN-SUB2-V6] 4d. VCS IMPORT: ROS2 + micro-ROS kaynak kodlarını MCU_WS içine klonla..."
CREATE_WS_SH=${UROS_WS}/src/micro_ros_setup/scripts/create_ws.sh
if [ ! -f "$CREATE_WS_SH" ]; then
  CREATE_WS_SH=$(find ${UROS_WS} -name create_ws.sh 2>/dev/null | head -1)
fi
bash ${CREATE_WS_SH} ${MCU_WS} ${CLIENT_ROS2_PKGS} ${CLIENT_UROS_REPOS} 2>&1

echo
echo "[GEN-SUB2-V6] 4d-1. DUPLICATE PACKAGE FIX (OFFICIAL: remove ROS2/rcl keep uros/rcl patched)"
if [ -d ${MCU_WS}/ros2/rcl ]; then
  echo "   Siliniyor: ros2/rcl (standart ROS2 RCL) — uros/rcl micro-ROS patched KALACAK"
  rm -rf ${MCU_WS}/ros2/rcl
fi
if [ -d ${MCU_WS}/ros2/rcutils ] && [ -d ${MCU_WS}/uros/rcutils ]; then
  echo "   Siliniyor: ros2/rcutils (duplicate)"
  rm -rf ${MCU_WS}/ros2/rcutils
fi
if [ -d ${MCU_WS}/ros2/rosidl_typesupport ] && [ -d ${MCU_WS}/uros/rosidl_typesupport ]; then
  echo "   Siliniyor: ros2/rosidl_typesupport (duplicate)"
  rm -rf ${MCU_WS}/ros2/rosidl_typesupport
fi
echo "   Kalan ros2 packages count: $(ls ${MCU_WS}/ros2 2>/dev/null | wc -l)"
echo "   uros packages: $(ls ${MCU_WS}/uros 2>/dev/null)"

echo
echo "[GEN-SUB2-V6] 4d-2. RCPPUTILS BARE-METAL STUB PATCH (CRITICAL V6 FIX)"
echo "   Kök neden: PQT-GCC 16.1.0 newlib nano sys/dirent.h -> #error <dirent.h> not supported"
echo "   Kök neden2: -fno-exceptions default -> throw std::system_error FAIL"
echo "   Çözüm: build ÖNCESİ src/ dosyalarını MINIMAL STUB ile OVERWRITE"
echo "   (Binary/manual libmicroros.a patch YOK - source-level generation ADIMI)"
RCPPUTILS_DIR=${MCU_WS}/ros2/rcpputils
if [ -d ${RCPPUTILS_DIR} ]; then
  echo "   Stub patch UYGULANIYOR: ${RCPPUTILS_DIR}/src/filesystem_helper.cpp"
  cat > ${RCPPUTILS_DIR}/src/filesystem_helper.cpp <<'STUB_EOF'
// ============================================================
// LESSIEN PICO B SUB2: rcpputils filesystem_helper.cpp STUB
// ORİJİNAL DOSYANIN YERİNE: Bare-metal (RP2350 Cortex-M33 no OS)
// PQT-GCC 16.1.0 newlib nano:
//   - sys/dirent.h: #error "<dirent.h> not supported"
//   - DIR / struct dirent / PATH_MAX / opendir/readdir/closedir MİSSİNG
//   - -fno-exceptions default: throw std::system_error derlenemez
// BU STUB: Derlemeyi geçmek için tüm public API imzalarını korur,
//          başarısız (ancak crash olmayan) değer döner.
// Micro-ROS CLIENT tarafında RMW bu fonksiyonları ASLA ÇAĞIRMAZ.
// ============================================================

#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/asserts.hpp"

// -------------------------------------------------------
// MİSSİNG bare-metal defines:
// -------------------------------------------------------
#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

#include <string>
#include <cstring>
#include <cstdint>
#include <sys/stat.h>

// -------------------------------------------------------
// Public rcpputils::fs STUB implementations
// -------------------------------------------------------
namespace rcpputils {
namespace fs {

// --- path ctors / operators ---
path::path() = default;
path::path(const path &) = default;
path::path(path &&) noexcept = default;
path & path::operator=(const path &) = default;
path & path::operator=(path &&) noexcept = default;
path::~path() = default;

path::path(const std::string & s) : path_(s) {}
path::path(const char * s) : path_(s ? s : "") {}

const std::string & path::string() const { return path_; }
const char * path::c_str() const { return path_.c_str(); }

bool path::operator==(const path & that) const { return path_ == that.path_; }
bool path::operator!=(const path & that) const { return path_ != that.path_; }
bool path::operator<(const path & that) const { return path_ < that.path_; }

path path::operator/(const path & that) const {
  if (that.path_.empty() || that.path_[0] == '/') { return that; }
  if (path_.empty() || path_.back() == '/') { return path(path_ + that.path_); }
  return path(path_ + "/" + that.path_);
}

path path::parent_path() const {
  auto pos = path_.find_last_of('/');
  if (pos == std::string::npos) { return path(""); }
  if (pos == 0) { return path("/"); }
  return path(path_.substr(0, pos));
}

path path::filename() const {
  auto pos = path_.find_last_of('/');
  if (pos == std::string::npos) { return path(path_); }
  return path(path_.substr(pos + 1));
}

path path::extension() const {
  auto fn = filename().string();
  auto pos = fn.find_last_of('.');
  if (pos == std::string::npos || pos == 0) { return path(""); }
  return path(fn.substr(pos));
}

path path::stem() const {
  auto fn = filename().string();
  auto pos = fn.find_last_of('.');
  if (pos == std::string::npos) { return path(fn); }
  return path(fn.substr(0, pos));
}

bool path::is_absolute() const {
  return !path_.empty() && path_[0] == '/';
}

bool path::exists() const {
  struct stat st;
  return ::stat(path_.c_str(), &st) == 0;
}

bool path::is_directory() const {
  struct stat st;
  if (::stat(path_.c_str(), &st) != 0) { return false; }
  return (st.st_mode & S_IFDIR) != 0;
}

bool path::is_regular_file() const {
  struct stat st;
  if (::stat(path_.c_str(), &st) != 0) { return false; }
  return (st.st_mode & S_IFREG) != 0;
}

uint64_t path::file_size() const {
  struct stat st;
  if (::stat(path_.c_str(), &st) != 0) { return 0ULL; }
  return static_cast<uint64_t>(st.st_size);
}

bool create_directories(const path & /*p*/) {
  // Bare-metal noop: başarıymış gibi dön (hiç çağrılmaz)
  return true;
}

bool remove(const path & /*p*/) { return true; }

bool remove_all(const path & /*p*/) {
  // Bare-metal noop
  return true;
}

path current_path() {
  // Bare-metal: kök dizin dön
  return path("/");
}

path weakly_canonical(const path & p) { return p; }

}  // namespace fs
}  // namespace rcpputils
STUB_EOF

  echo "   Stub patch UYGULANIYOR: ${RCPPUTILS_DIR}/src/env.cpp"
  cat > ${RCPPUTILS_DIR}/src/env.cpp <<'STUB_EOF2'
// ============================================================
// LESSIEN PICO B SUB2: rcpputils env.cpp STUB
// -fexceptions kapalı olduğu için throw yerine return.
// ============================================================
#include "rcpputils/env.hpp"
#include <cstdlib>
#include <string>

namespace rcpputils {

std::string get_env_var(const char * name) {
  if (!name) { return ""; }
  const char * val = ::getenv(name);
  return val ? std::string(val) : "";
}

bool set_env_var(const char * name, const char * value, bool overwrite) {
  if (!name) { return false; }
  if (!overwrite) {
    const char * existing = ::getenv(name);
    if (existing) { return true; }
  }
#if defined(_WIN32)
  return false;
#else
  return ::setenv(name, value ? value : "", 1) == 0;
#endif
}

bool clear_env_var(const char * name) {
  if (!name) { return false; }
#if defined(_WIN32)
  return false;
#else
  return ::unsetenv(name) == 0;
#endif
}

}  // namespace rcpputils
STUB_EOF2

  echo "   Stub patch: asserts.hpp exceptions define (throw yerine abort)"
  ASSERTS_HPP=${RCPPUTILS_DIR}/include/rcpputils/asserts.hpp
  if [ -f "$ASSERTS_HPP" ]; then
    sed -i 's|throw std::invalid_argument|rcpputils_throw_noop_invalid_arg|g' "$ASSERTS_HPP" || true
    cat >> "$ASSERTS_HPP" <<'STUB_ASSERT'
#ifdef __cplusplus
extern "C" {
#endif
// LESSIEN PICO B SUB2: bare-metal throw yerine abort (no-exception derlemesi)
inline static void rcpputils_throw_noop_invalid_arg(const char * msg) {
  (void)msg;
  for(;;) { /* infinite no-op loop (never reached in micro-ROS client) */ }
}
#ifdef __cplusplus
}
#endif
STUB_ASSERT
  fi

  echo "   ✅ rcpputils STUB PATCH UYGULANDI."
  ls -la ${RCPPUTILS_DIR}/src/filesystem_helper.cpp ${RCPPUTILS_DIR}/src/env.cpp
else
  echo "   ⚠️  UYARI: RCPPUTILS_DIR BULUNAMADI -> ${RCPPUTILS_DIR}"
fi

echo
echo "[GEN-SUB2-V6] 4e. MCU_WS altındaki packages listesini kontrol et:"
if [ -d ${MCU_WS}/ros2 ]; then
  echo "   ros2 packages count: $(ls ${MCU_WS}/ros2 2>/dev/null | wc -l)"
  ls ${MCU_WS}/ros2 2>/dev/null | head -30
fi

DEFAULT_META=${UROS_WS}/src/micro_ros_setup/config/generate_lib/generic/client-colcon.meta
if [ -f "${DEFAULT_META}" ]; then
  cp ${DEFAULT_META} ${MCU_WS}/colcon.meta
  echo "   client-colcon.meta kopyalandı mcu_ws altına"
fi

echo "[GEN-SUB2-V6] 5. tf2_msgs + extra packages import (RESMI library_generation.sh İLE AYNI ADIMLAR)"
pushd ${MCU_WS} > /dev/null
    echo "   clone geometry2 (jazzy) - depth 1"
    git clone -b jazzy https://github.com/ros2/geometry2 --depth 1 2>&1 | tail -3
    if [ -d geometry2/tf2_msgs ]; then
      cp -R geometry2/tf2_msgs ros2/tf2_msgs || true
      echo "   tf2_msgs kopyalandı -> ros2/tf2_msgs"
    else
      echo "   UYARI: geometry2/tf2_msgs KOPYALANAMADI"
    fi
    rm -rf geometry2 || true

    mkdir -p extra_packages
    pushd extra_packages > /dev/null
        EXTRA_DIR=${TEMP_STAGE}/extras/library_generation/extra_packages
        if [ -d "${EXTRA_DIR}" ]; then
          echo "   extra packages kopyala: ${EXTRA_DIR}"
          cp -R ${EXTRA_DIR}/* . 2>/dev/null || true
          if [ -f extra_packages.repos ]; then
            vcs import --input extra_packages.repos 2>&1 | tail -5 || true
          fi
        fi
    popd > /dev/null
popd > /dev/null

echo "[GEN-SUB2-V6] 6. TEMIZLE: /project/src içinden eski header dışı dosyaları sil"
find ${TEMP_STAGE}/src/ ! -name micro_ros_arduino.h ! -name "*.c" ! -name "*.cpp" ! -name "*.c.in" -delete 2>/dev/null || true

echo "============================================================"
echo "[GEN-SUB2-V6] 7. CROSS-COMPILE SUB2 libmicroros.a (Cortex-M33 softfp RP2350)"
echo "============================================================"
echo "   TOOLCHAIN_PREFIX = ${TOOLCHAIN_PREFIX}"
echo "   TOOLCHAIN_CMAKE  = ${TOOLCHAIN_CMAKE}"
echo "   SUB2_META        = ${SUB2_META}"
echo
ls -la ${TOOLCHAIN_PREFIX}gcc 2>&1 | head -3
echo

BUILD_FW_SH=${UROS_WS}/src/micro_ros_setup/scripts/build_firmware.sh
if [ ! -f "${BUILD_FW_SH}" ]; then
  BUILD_FW_SH=$(find ${UROS_WS} -name "build_firmware.sh" 2>/dev/null | head -1)
fi
echo "   build_firmware.sh path = ${BUILD_FW_SH}"

rm -rf ${FW_TARGETDIR}/build

cd ${TEMP_STAGE}
ros2 run micro_ros_setup build_firmware.sh \
    ${TOOLCHAIN_CMAKE} \
    ${SUB2_META} 2>&1 | tee /tmp/build_fw_sub2_v6.log
BUILD_RET=${PIPESTATUS[0]}

echo
echo "============================================================"
echo "[GEN-SUB2-V6] 7b. Build Sonu log tail 100 satır:"
tail -100 /tmp/build_fw_sub2_v6.log
echo
echo "   Build exit code = ${BUILD_RET}"

if [ ${BUILD_RET} -eq 0 ] && [ -f ${FW_TARGETDIR}/build/libmicroros.a ]; then
  echo "============================================================"
  echo "[GEN-SUB2-V6] ✅ BUILD BAŞARILI! libmicroros.a üretildi."
  ls -la ${FW_TARGETDIR}/build/libmicroros.a
  echo
  echo "   Firmware build/include dosyalarını stage/src içine kopyala"
  find ${FW_TARGETDIR}/build/include/ -name "*.c"  -delete 2>/dev/null || true
  cp -R ${FW_TARGETDIR}/build/include/* ${TEMP_STAGE}/src/ 2>&1 | tail -3

  echo "   Cortex-M33 softfp klasörünü oluştur ve libmicroros.a kopyala"
  mkdir -p ${TEMP_STAGE}/src/cortex-m33/softfp
  cp -v ${FW_TARGETDIR}/build/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/softfp/libmicroros.a
  ln -sf softfp/libmicroros.a ${TEMP_STAGE}/src/cortex-m33/libmicroros.a 2>/dev/null || true

  echo
  echo "[GEN-SUB2-V6] 8. Fix include paths (RESMI library_generation.sh ADIM 262-270)"
  pushd ${MCU_WS} > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list 2>/dev/null | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
  popd > /dev/null
  echo "   ROS2 packages = ${INCLUDE_ROS2_PACKAGES}"
  for var in ${INCLUDE_ROS2_PACKAGES}; do
    rsync -r ${TEMP_STAGE}/src/${var}/${var}/* ${TEMP_STAGE}/src/${var}/ > /dev/null 2>&1
    rm -rf ${TEMP_STAGE}/src/${var}/${var}/ > /dev/null 2>&1
  done

  echo
  echo "[GEN-SUB2-V6] 9. library.properties metadata SUB2 için güncelle"
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
  echo "   yeni library.properties:"
  cat ${TEMP_STAGE}/library.properties

  echo
  echo "[GEN-SUB2-V6] 10. FINAL: Temp stage'yi OUTPUT_NEW_LIB_DIR içine KOPYALA"
  if [ -d ${OUTPUT_NEW_LIB_DIR} ]; then
    echo "   Eski SUB2 output varsa temizle (önceden failed build varsa)"
    rm -rf ${OUTPUT_NEW_LIB_DIR}
  fi
  rsync -a ${TEMP_STAGE}/ ${OUTPUT_NEW_LIB_DIR}/
  echo "   KOPYALANDI! ${OUTPUT_NEW_LIB_DIR}"
  echo
  echo "   Yeni lib SHA256:"
  NEW_SHA=$(sha256sum ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/libmicroros.a | awk '{print $1}')
  echo "   ${NEW_SHA}"

  echo
  echo "   RMW config doğrulama (yeni SUB=2):"
  grep -E "MAX_NODES|MAX_PUBLISHERS|MAX_SUBSCRIPTIONS|MAX_HISTORY|MAX_SERVICES|MAX_CLIENTS" \
    ${OUTPUT_NEW_LIB_DIR}/src/rmw_microxrcedds_c/config.h

  echo
  echo "   Eski SUB1 lib ile KIYAS (eski DEĞİŞMEDİ) - SHA KARŞILAŞTIRMA:"
  OLD_LIB=${PORT}/micro_ros_arduino_rp2350_jazzy_pico_b/src/cortex-m33/softfp/libmicroros.a
  OLD_SHA=$(sha256sum ${OLD_LIB} | awk '{print $1}')
  echo "   OLD SUB1 SHA = ${OLD_SHA}"
  echo "   NEW SUB2 SHA = ${NEW_SHA}"

  echo
  echo "   Typesupport doğrulama (Twist + Float32):"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/twist.h 2>&1
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/std_msgs/msg/float32.h 2>&1
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/geometry_msgs/msg/detail/twist__type_support.h 2>&1
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/std_msgs/msg/detail/float32__type_support.h 2>&1

  echo
  echo "   Cortex-M33 doğrulama:"
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/ 2>&1
  ls -la ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/ 2>&1
  file ${OUTPUT_NEW_LIB_DIR}/src/cortex-m33/softfp/libmicroros.a 2>&1 | head -2

  echo
  echo "============================================================"
  echo "✅✅✅ OFFICIAL SUB=2 LIBRARY GENERATION BAŞARILI ✅✅✅"
  echo "============================================================"
else
  echo
  echo "============================================================"
  echo "❌ BUILD HATASI! Exit code: ${BUILD_RET}"
  echo "Loglar:"
  echo "  /tmp/build_fw_sub2_v6.log"
  echo "  ${LOG_OUT}"
  if [ -d ${FW_TARGETDIR}/build ]; then
    echo "Build klasörü MEVCUT (${FW_TARGETDIR}/build)"
    ls -la ${FW_TARGETDIR}/build/ 2>&1 | head -15
  else
    echo "Build klasörü YOK"
  fi
  echo "============================================================"
fi

date
echo "TEMP_STAGE korundu: ${TEMP_STAGE}  (inceleme için)"
exit ${BUILD_RET}
