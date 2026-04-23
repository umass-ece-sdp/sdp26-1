#!/usr/bin/env bash
set -e

# In the new architecture, base station is a client to both:
# 1. Tello drone (on one WiFi interface)
# 2. ESP32 glove AP (on second WiFi interface, optional)
# 
# This script simply disconnects both interfaces gracefully.

# Detect all available WiFi interfaces
mapfile -t WIFI_IFACES < <(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi" {print $1}')

if [ "${#WIFI_IFACES[@]}" -eq 0 ]; then
    echo "No WiFi interface detected."
    exit 0
fi

echo "Disconnecting WiFi interfaces..."
for WIFI_IFACE in "${WIFI_IFACES[@]}"; do
    echo "  Disconnecting $WIFI_IFACE..."
    nmcli device disconnect "$WIFI_IFACE" 2>/dev/null || true
done

echo "✓ WiFi cleanup complete."
echo "  All connections have been disconnected."
