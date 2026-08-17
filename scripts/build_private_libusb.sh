#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 INSTALL_PREFIX" >&2
  exit 2
fi

install_prefix=$1
libusb_version=1.0.27
archive_sha256=ffaa41d741a8a3bee244ac8e54a72ea05bf2879663c098c82fc5757853441575
archive_name="libusb-${libusb_version}.tar.bz2"
archive_url="https://github.com/libusb/libusb/releases/download/v${libusb_version}/${archive_name}"
work_dir=$(mktemp -d)

cleanup() {
  rm -rf -- "$work_dir"
}
trap cleanup EXIT

curl --fail --location --silent --show-error \
  --output "$work_dir/$archive_name" "$archive_url"
printf '%s  %s\n' "$archive_sha256" "$work_dir/$archive_name" | sha256sum --check --status
tar -xf "$work_dir/$archive_name" -C "$work_dir"

pushd "$work_dir/libusb-${libusb_version}" >/dev/null
configure_host=()
if [[ -n "${MOTOR_LIBUSB_CONFIGURE_HOST:-}" ]]; then
  configure_host=("--host=${MOTOR_LIBUSB_CONFIGURE_HOST}")
fi
./configure \
  "${configure_host[@]}" \
  --prefix="$install_prefix" \
  --disable-static \
  --enable-shared \
  --disable-udev
make -j"$(getconf _NPROCESSORS_ONLN)"
make install
popd >/dev/null

support_library="$install_prefix/lib/libusb-1.0.so.0"
test -f "$support_library"
nm -D "$support_library" | grep -q ' libusb_init_context$'
