#!/usr/bin/env bash
# Reset WiFi connections for FALCON system
# Hardcoded values - no user input needed

set -e

echo "=========================================="
echo "FALCON WiFi Reset"
echo "=========================================="

# Detect all available WiFi interfaces
mapfile -t WIFI_IFACES < <(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi" {print $1}')

if [ "${#WIFI_IFACES[@]}" -eq 0 ]; then
    echo "⚠ No WiFi interface detected."
    exit 0
fi

echo "Disconnecting WiFi interfaces..."
for WIFI_IFACE in "${WIFI_IFACES[@]}"; do
    echo "  Disconnecting $WIFI_IFACE..."
    nmcli device disconnect "$WIFI_IFACE" 2>/dev/null || true
done

echo ""
echo "=========================================="
echo "✓ WiFi Cleanup Complete"
echo "=========================================="
echo "All connections have been disconnected."
echo "=========================================="
