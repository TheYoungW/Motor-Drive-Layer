#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=${MOTOR_AARCH64_BUILD_DIR:-"${repo_dir}/build-aarch64"}
install_dir=${MOTOR_AARCH64_INSTALL_DIR:-"${build_dir}/stage"}
package_dir=${MOTOR_AARCH64_PACKAGE_DIR:-"${repo_dir}/dist/arm64"}

: "${MOTOR_AARCH64_SYSROOT:?set MOTOR_AARCH64_SYSROOT to the RK3588 sysroot}"
: "${ARTICORE_HOST_IDLC:?set ARTICORE_HOST_IDLC to host Cyclone DDS 11.0.1 idlc}"
: "${ARTICORE_PINOCCHIO_HEADER_ROOT:?set ARTICORE_PINOCCHIO_HEADER_ROOT to the Pinocchio 3.8.0 include root}"

cyclone_dir=${ARTICORE_CYCLONEDDS_ARM64_DIR:-"${MOTOR_AARCH64_SYSROOT}/usr/lib/aarch64-linux-gnu/cmake/CycloneDDS"}

cmake \
  -S "${repo_dir}" \
  -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DARTICORE_ENABLE_DDS=ON \
  -DARTICORE_HOST_IDLC="${ARTICORE_HOST_IDLC}" \
  -DARTICORE_PINOCCHIO_HEADER_ROOT="${ARTICORE_PINOCCHIO_HEADER_ROOT}" \
  -DCycloneDDS_DIR="${cyclone_dir}" \
  -DCMAKE_TOOLCHAIN_FILE="${repo_dir}/cmake/toolchains/linux-aarch64.cmake" \
  -DCMAKE_INSTALL_PREFIX="${install_dir}" \
  -DCPACK_PACKAGE_DIRECTORY="${package_dir}"
cmake --build "${build_dir}" --parallel
cmake --install "${build_dir}"
cmake --build "${build_dir}" --target package

printf '%s\n' "Staged RK3588/aarch64 Runtime Service under ${install_dir}"
printf '%s\n' "ARM64 Debian package written under ${package_dir}"
