#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"

PKG=ups-monitor
VER=1.0.0
OUT="${PKG}_${VER}_all.deb"
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

mkdir -p "$STAGE"/{usr/lib/ups,lib/systemd/system,etc/modules-load.d,DEBIAN}
cp ltc3350_driver.py monitor-ups.py monitor-ups-health.py "$STAGE/usr/lib/ups/"
cp systemd/ups.service systemd/ups-health.service systemd/ups-health.timer "$STAGE/lib/systemd/system/"
printf 'i2c-dev\n' > "$STAGE/etc/modules-load.d/i2c.conf"

cat > "$STAGE/DEBIAN/control" <<EOF
Package: $PKG
Version: $VER
Section: misc
Priority: optional
Architecture: all
Maintainer: James Lucas <james@edgetelemetrics.com.au>
Depends: python3 (>= 3.7), python3-smbus, i2c-tools
Description: LTC3350 SuperCap UPS monitor for Raspberry Pi
EOF

printf '/etc/modules-load.d/i2c.conf\n' > "$STAGE/DEBIAN/conffiles"

cat > "$STAGE/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e
install -d -m 0750 /var/lib/ups
systemctl daemon-reload
systemctl enable ups.service ups-health.timer
systemctl restart ups.service
modprobe i2c-dev 2>/dev/null || true
EOF

cat > "$STAGE/DEBIAN/prerm" <<'EOF'
#!/bin/sh
set -e
systemctl stop ups.service 2>/dev/null || true
systemctl disable ups.service 2>/dev/null || true
systemctl stop ups-health.timer 2>/dev/null || true
systemctl disable ups-health.timer 2>/dev/null || true
EOF

chmod 0755 "$STAGE/DEBIAN/postinst" "$STAGE/DEBIAN/prerm"

dpkg-deb --root-owner-group --build "$STAGE" "$OUT"
echo "built $OUT"
