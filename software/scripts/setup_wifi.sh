#!/usr/bin/env bash
# Setup WiFi for FALCON system - connects to both Tello and glove AP
# Hardcoded values - no user input needed

set -e

# Hardcoded WiFi credentials
TELLO_SSID="TELLO-FE046A"
TELLO_PASSWORD=""
GLOVE_SSID="FALCON-Glove"
GLOVE_PASSWORD="team1-falcon"

echo "=========================================="
echo "FALCON WiFi Setup (Hardcoded)"
echo "=========================================="
echo "Tello SSID: $TELLO_SSID"
echo "Glove SSID: $GLOVE_SSID"
echo ""

# Detect available WiFi interfaces
mapfile -t WIFI_IFACES < <(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi" {print $1}')

if [ "${#WIFI_IFACES[@]}" -eq 0 ]; then
    echo "❌ Error: No WiFi interface detected."
    exit 1
fi

if [ "${#WIFI_IFACES[@]}" -lt 2 ]; then
    echo "❌ Error: Only ${#WIFI_IFACES[@]} WiFi interface(s) found."
    echo "   This project requires TWO WiFi interfaces."
    exit 1
fi

TELLO_IFACE="${WIFI_IFACES[0]}"
GLOVE_IFACE="${WIFI_IFACES[1]}"

echo "Detected WiFi interfaces: ${WIFI_IFACES[*]}"
echo "  - Tello interface: $TELLO_IFACE"
echo "  - Glove interface: $GLOVE_IFACE"
echo ""

# Connect to Tello drone
echo "[1/2] Connecting $TELLO_IFACE to Tello..."
nmcli device connect "$TELLO_IFACE" >/dev/null 2>&1 || true
nmcli device wifi connect "$TELLO_SSID" password "$TELLO_PASSWORD" ifname "$TELLO_IFACE"
echo "✓ Tello connection established on $TELLO_IFACE"
sleep 2

# Connect to ESP32 glove AP
echo ""
echo "[2/2] Connecting $GLOVE_IFACE to glove AP..."

# Remove any existing connections on this interface to start fresh
mapfile -t CONNS < <(nmcli -t -f NAME,DEVICE connection show | grep "$GLOVE_IFACE" | awk -F: '{print $1}')
for conn in "${CONNS[@]}"; do
    echo "  Removing existing connection: $conn"
    nmcli connection delete "$conn" 2>/dev/null || true
done

# Connect to glove AP
nmcli device connect "$GLOVE_IFACE" >/dev/null 2>&1 || true
nmcli device wifi connect "$GLOVE_SSID" password "$GLOVE_PASSWORD" ifname "$GLOVE_IFACE"
echo "✓ Glove AP connection established on $GLOVE_IFACE"
sleep 2

# Verify connection - get IP assigned by glove's AP
ASSIGNED_IP=$(nmcli -t -f IP4.ADDRESS device show "$GLOVE_IFACE" | grep -oP '192\.168\.4\.\d+' | head -1)
if [ -n "$ASSIGNED_IP" ]; then
    echo "✓ Assigned IP from glove AP: $ASSIGNED_IP"
else
    echo "⚠ Could not verify assigned IP, but connection should be established."
fi

echo ""
echo "=========================================="
echo "✓ WiFi Setup Complete!"
echo "=========================================="
echo "  Tello interface: $TELLO_IFACE"
echo "  Glove interface: $GLOVE_IFACE"
echo "  Glove server: 192.168.4.1:5000"
echo ""
echo "Ready to run: python -m software.src.main"
echo "=========================================="
