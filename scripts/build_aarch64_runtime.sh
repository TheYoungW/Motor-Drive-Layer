#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=${MOTOR_AARCH64_BUILD_DIR:-"${repo_dir}/build-aarch64"}
install_dir=${MOTOR_AARCH64_INSTALL_DIR:-"${repo_dir}/dist/aarch64"}

cmake \
  -S "${repo_dir}" \
  -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DCMAKE_TOOLCHAIN_FILE="${repo_dir}/cmake/toolchains/linux-aarch64.cmake" \
  -DCMAKE_INSTALL_PREFIX="${install_dir}"
cmake --build "${build_dir}" --parallel
cmake --install "${build_dir}"

if [[ "${MOTOR_AARCH64_BUNDLE_DM_DEVICE:-1}" != "0" ]]; then
  support_dir="${build_dir}/private-libusb"
  MOTOR_LIBUSB_CONFIGURE_HOST=aarch64-linux-gnu \
  CC=aarch64-linux-gnu-gcc \
    "${repo_dir}/scripts/build_private_libusb.sh" "${support_dir}"
  cmake -E make_directory "${install_dir}/lib/dm_device"
  cmake -E copy \
    "${support_dir}/lib/libusb-1.0.so.0" \
    "${install_dir}/lib/dm_device/libusb-1.0.so.0"
fi

printf '%s\n' "Installed RK3588/aarch64 native Runtime under ${install_dir}"
