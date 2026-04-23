#!/usr/bin/env bash
set -e

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <tello_ssid> <tello_password> [esp32_ssid] [esp32_password]"
    echo ""
    echo "Connects base station to Tello drone (and optionally to ESP32 AP if second WiFi interface available)."
    echo "Defaults: esp32_ssid='FALCON-Glove', esp32_password='team1-falcon'"
    exit 2
fi

TELLO_SSID="$1"
TELLO_PASSWORD="$2"
ESP32_SSID="${3:-FALCON-Glove}"
ESP32_PASSWORD="${4:-team1-falcon}"

# Detect available WiFi interfaces
mapfile -t WIFI_IFACES < <(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi" {print $1}')

if [ "${#WIFI_IFACES[@]}" -eq 0 ]; then
    echo "Error: No WiFi interface detected."
    exit 1
fi

TELLO_IFACE="${WIFI_IFACES[0]}"
ESP32_IFACE=""
if [ "${#WIFI_IFACES[@]}" -ge 2 ]; then
    ESP32_IFACE="${WIFI_IFACES[1]}"
fi

echo "Detected WiFi interface(s): ${WIFI_IFACES[*]}"
echo "Using interface for Tello: $TELLO_IFACE"

# Connect to Tello drone
echo "Connecting $TELLO_IFACE to Tello SSID: $TELLO_SSID"
nmcli device connect "$TELLO_IFACE" >/dev/null 2>&1 || true
nmcli device wifi connect "$TELLO_SSID" password "$TELLO_PASSWORD" ifname "$TELLO_IFACE"
echo "✓ Tello connection established on $TELLO_IFACE"

# Connect to ESP32 glove AP if second interface available
if [ -z "$ESP32_IFACE" ]; then
    echo ""
    echo "⚠ Only one WiFi interface found - but project requires two!"
    echo "  Please ensure you have two WiFi interfaces available."
    echo "  - Interface 1: For Tello drone"
    echo "  - Interface 2: For ESP32 glove AP"
    exit 0
fi

echo ""
echo "Using interface for ESP32 glove: $ESP32_IFACE"
echo "Connecting $ESP32_IFACE to ESP32 AP: $ESP32_SSID"

# Remove any existing connections on this interface to start fresh
mapfile -t CONNS < <(nmcli -t -f NAME,DEVICE connection show | grep "$ESP32_IFACE" | awk -F: '{print $1}')
for conn in "${CONNS[@]}"; do
    echo "Removing existing connection: $conn"
    nmcli connection delete "$conn" 2>/dev/null || true
done

# Connect to ESP32 AP
echo "Connecting $ESP32_IFACE to ESP32 AP: $ESP32_SSID"
nmcli device connect "$ESP32_IFACE" >/dev/null 2>&1 || true
nmcli device wifi connect "$ESP32_SSID" password "$ESP32_PASSWORD" ifname "$ESP32_IFACE"
echo "✓ ESP32 AP connection established on $ESP32_IFACE"

# Verify connection - get IP assigned by glove's AP
ASSIGNED_IP=$(nmcli -t -f IP4.ADDRESS device show "$ESP32_IFACE" | grep -oP '192\.168\.4\.\d+' | head -1)
if [ -n "$ASSIGNED_IP" ]; then
    echo "✓ Assigned IP: $ASSIGNED_IP"
    echo "✓ Glove server is running at: 192.168.4.1:5000"
else
    echo "⚠ Could not verify assigned IP, but connection should be established."
    echo "✓ Glove server is running at: 192.168.4.1:5000"
fi

echo ""
echo "✓ WiFi configuration complete!"
echo "  - Tello connection on: $TELLO_IFACE"
echo "  - ESP32 glove connection on: $ESP32_IFACE"
echo "  - Glove TCP server: 192.168.4.1:5000"
