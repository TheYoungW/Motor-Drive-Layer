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

printf '%s\n' "Installed RK3588/aarch64 native Runtime under ${install_dir}"
