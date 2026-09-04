#!/bin/sh
set -eu

root=$1
temporary=$(mktemp -d)
trap 'rm -rf "$temporary"' EXIT HUP INT TERM

mkdir -p "$temporary/sys/class/net/can0" "$temporary/sys/class/net/can1" \
  "$temporary/bin"
printf '%s\n' LEFT123 >"$temporary/sys/class/net/can0/serial"
printf '%s\n' RIGHT456 >"$temporary/sys/class/net/can1/serial"

cat >"$temporary/bin/udevadm" <<'EOF'
#!/bin/sh
for argument in "$@"; do path=$argument; done
printf 'ID_SERIAL_SHORT=%s\n' "$(cat "$path/serial")"
EOF

cat >"$temporary/bin/ip" <<'EOF'
#!/bin/sh
printf '%s\n' "$*" >>"$TEST_LOG"
if [ "$1:$2:$3:$5" = "link:set:dev:name" ]; then
  mv "$TEST_SYS_CLASS_NET/$4" "$TEST_SYS_CLASS_NET/$6"
fi
EOF
chmod 0755 "$temporary/bin/udevadm" "$temporary/bin/ip"

cat >"$temporary/can.conf" <<'EOF'
LEFT_CAN_INTERFACE=can-left
RIGHT_CAN_INTERFACE=can-right
LEFT_CAN_SERIAL=LEFT123
RIGHT_CAN_SERIAL=RIGHT456
CAN_BITRATE=1000000
CAN_DBITRATE=5000000
CAN_SAMPLE_POINT=0.80
CAN_DSAMPLE_POINT=0.875
CAN_RESTART_MS=100
EOF

TEST_LOG="$temporary/ip.log" \
TEST_SYS_CLASS_NET="$temporary/sys/class/net" \
ARTICORE_SYS_CLASS_NET="$temporary/sys/class/net" \
ARTICORE_IP="$temporary/bin/ip" \
ARTICORE_UDEVADM="$temporary/bin/udevadm" \
  "$root/deploy/articore-can-init" "$temporary/can.conf"

test -d "$temporary/sys/class/net/can-left"
test -d "$temporary/sys/class/net/can-right"
grep -q 'dev can-left type can bitrate 1000000 sample-point 0.80 dbitrate 5000000 dsample-point 0.875 fd on restart-ms 100' "$temporary/ip.log"
grep -q 'dev can-right type can bitrate 1000000 sample-point 0.80 dbitrate 5000000 dsample-point 0.875 fd on restart-ms 100' "$temporary/ip.log"
if grep -q 'berr-reporting' "$temporary/ip.log"; then
  echo "CAN init must not enable berr-reporting" >&2
  exit 1
fi

cat >"$temporary/bin/systemctl" <<'EOF'
#!/bin/sh
printf '%s\n' "$*" >>"$TEST_LOG"
if [ "$1" = is-active ]; then
  [ "$RUNTIME_ACTIVE" = yes ]
fi
EOF
cat >"$temporary/bin/logger" <<'EOF'
#!/bin/sh
printf '%s\n' "$*" >>"$TEST_LOG"
EOF
chmod 0755 "$temporary/bin/systemctl" "$temporary/bin/logger"

TEST_LOG="$temporary/hotplug-active.log" RUNTIME_ACTIVE=yes \
ARTICORE_SYSTEMCTL="$temporary/bin/systemctl" \
ARTICORE_LOGGER="$temporary/bin/logger" ARTICORE_HOTPLUG_DELAY=0 \
  "$root/deploy/articore-can-hotplug"
grep -q '^restart articore-can.service$' "$temporary/hotplug-active.log"
grep -q 'Runtime was active before hotplug and will not be auto-started' "$temporary/hotplug-active.log"
if grep -q '^start .*articore-runtime.service$' "$temporary/hotplug-active.log"; then
  echo "hotplug must not restart or start an active Runtime" >&2
  exit 1
fi

TEST_LOG="$temporary/hotplug-inactive.log" RUNTIME_ACTIVE=no \
ARTICORE_SYSTEMCTL="$temporary/bin/systemctl" \
ARTICORE_LOGGER="$temporary/bin/logger" ARTICORE_HOTPLUG_DELAY=0 \
  "$root/deploy/articore-can-hotplug"
grep -q '^restart articore-can.service$' "$temporary/hotplug-inactive.log"
grep -q '^start --no-block articore-runtime.service$' "$temporary/hotplug-inactive.log"

grep -q '^dds_interfaces=eth0,eth1,wlan0$' "$root/deploy/runtime-service.conf"
grep -q 'LEFT_CAN_SERIAL=AEEDE4FD23DEA4AFCA6B3EAA55ABC28A' "$root/deploy/can.conf"
grep -q 'RIGHT_CAN_SERIAL=015213EF68D8345BBAA6D57818A4EC3A' "$root/deploy/can.conf"
grep -q 'ATTRS{idVendor}=="1d50"' "$root/deploy/99-articore-can-hotplug.rules"
grep -q 'ATTRS{idProduct}=="606f"' "$root/deploy/99-articore-can-hotplug.rules"
grep -q 'Requires=articore-can.service' "$root/deploy/articore-runtime.service"
grep -q '^ExecStartPre=/usr/libexec/articore/articore-wait-dds-network /etc/articore/runtime-service.conf$' "$root/deploy/articore-runtime.service"
grep -q '^TimeoutStartSec=infinity$' "$root/deploy/articore-runtime.service"
grep -q '^CPUAffinity=0 1 2 3 4$' "$root/deploy/articore-runtime.service"
grep -q '^CPUSchedulingPolicy=other$' "$root/deploy/articore-runtime.service"
grep -q 'Environment=ARTICORE_RUNTIME_CONTROL_HZ=500' "$root/deploy/articore-runtime.service"
grep -q '^can_rx_priority=75$' "$root/deploy/runtime-service.conf"
grep -q 'systemctl enable articore-can.service articore-runtime.service' "$root/deploy/debian/postinst"

mkdir -p "$temporary/network-sys" "$temporary/network-bin"
cat >"$temporary/network.conf" <<'EOF'
dds_interfaces=wlan0
EOF
cat >"$temporary/network-bin/ip" <<'EOF'
#!/bin/sh
case "$*" in
  '-o link show dev wlan0')
    if [ -e "$TEST_NETWORK_LINK_READY" ]; then
      printf '%s\n' '7: wlan0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500'
    else
      printf '%s\n' '7: wlan0: <BROADCAST,MULTICAST,UP> mtu 1500'
    fi
    ;;
  '-o address show dev wlan0 scope global')
    [ -e "$TEST_NETWORK_READY" ] &&
      printf '%s\n' '7: wlan0 inet 192.168.1.185/24 scope global wlan0'
    ;;
esac
EOF
cat >"$temporary/network-bin/sleep" <<'EOF'
#!/bin/sh
printf '%s\n' "$1" >>"$TEST_NETWORK_SLEEP_LOG"
if [ ! -e "$TEST_NETWORK_SYS/wlan0" ]; then
  mkdir -p "$TEST_NETWORK_SYS/wlan0"
elif [ ! -e "$TEST_NETWORK_LINK_READY" ]; then
  : >"$TEST_NETWORK_LINK_READY"
else
  : >"$TEST_NETWORK_READY"
fi
EOF
chmod 0755 "$temporary/network-bin/ip" "$temporary/network-bin/sleep"

TEST_NETWORK_READY="$temporary/network-ready" \
TEST_NETWORK_LINK_READY="$temporary/network-link-ready" \
TEST_NETWORK_SLEEP_LOG="$temporary/network-sleep.log" \
TEST_NETWORK_SYS="$temporary/network-sys" \
ARTICORE_SYS_CLASS_NET="$temporary/network-sys" \
ARTICORE_IP="$temporary/network-bin/ip" \
ARTICORE_SLEEP="$temporary/network-bin/sleep" \
  "$root/deploy/articore-wait-dds-network" "$temporary/network.conf" \
  >"$temporary/network-wait.out" 2>"$temporary/network-wait.err"
grep -q 'wlan0 is not present; waiting' "$temporary/network-wait.err"
grep -q 'wlan0 link is not connected; waiting' "$temporary/network-wait.err"
grep -q 'wlan0 has no global IP address; waiting' "$temporary/network-wait.err"
test "$(wc -l <"$temporary/network-sleep.log")" -eq 3
grep -q '^DDS network ready on wlan0$' "$temporary/network-wait.out"

# With multiple configured interfaces, one connected interface is sufficient;
# Cyclone marks the unavailable backup as optional in its generated XML.
cat >"$temporary/network-any.conf" <<'EOF'
dds_interfaces=eth0,wlan0
EOF
ARTICORE_SYS_CLASS_NET="$temporary/network-sys" \
ARTICORE_IP="$temporary/network-bin/ip" \
ARTICORE_SLEEP="$temporary/network-bin/sleep" \
TEST_NETWORK_READY="$temporary/network-ready" \
TEST_NETWORK_LINK_READY="$temporary/network-link-ready" \
TEST_NETWORK_SLEEP_LOG="$temporary/network-sleep.log" \
TEST_NETWORK_SYS="$temporary/network-sys" \
  "$root/deploy/articore-wait-dds-network" "$temporary/network-any.conf" \
  >"$temporary/network-any.out" 2>"$temporary/network-any.err"
grep -q '^DDS network ready on wlan0$' "$temporary/network-any.out"
test ! -s "$temporary/network-any.err"

cat >"$temporary/bin/getent" <<'EOF'
#!/bin/sh
exit 0
EOF
chmod 0755 "$temporary/bin/getent"
mkdir -p "$temporary/systemd/articore-can.service.d" "$temporary/migration"
cat >"$temporary/systemd/articore-can-hotplug.service" <<'EOF'
[Unit]
Description=Reconfigure Articore CAN interfaces after USB hotplug

[Service]
Type=oneshot
ExecStart=/usr/local/libexec/articore/articore-can-hotplug
EOF
cat >"$temporary/systemd/articore-can.service.d/override.conf" <<'EOF'
[Service]
ExecStart=
ExecStart=/usr/local/libexec/articore/articore-can-init
EOF

PATH="$temporary/bin:$PATH" TEST_LOG="$temporary/postinst.log" \
RUNTIME_ACTIVE=no ARTICORE_SYSTEMD_ETC_DIR="$temporary/systemd" \
ARTICORE_MIGRATION_BACKUP_DIR="$temporary/migration" \
  sh "$root/deploy/debian/postinst" configure

test ! -e "$temporary/systemd/articore-can-hotplug.service"
test ! -e "$temporary/systemd/articore-can.service.d/override.conf"
test -f "$temporary/migration/articore-can-hotplug.service"
test -f "$temporary/migration/articore-can.service.override.conf"
grep -q '^enable articore-can.service articore-runtime.service$' "$temporary/postinst.log"
grep -q '^restart articore-can.service$' "$temporary/postinst.log"
grep -q '^start --no-block articore-runtime.service$' "$temporary/postinst.log"
