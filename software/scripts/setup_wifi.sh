#!/usr/bin/env bash
set -e

# Detect the active WiFi interface
WIFI_IFACE=$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi" {print $1}' | head -n 1)

if [ -z "$WIFI_IFACE" ]; then
    echo "Error: No WiFi interface detected."
    exit 1
fi

echo "Detected WiFi interface: $WIFI_IFACE"

# Check if connection already exists and remove it to start fresh
if nmcli connection show "jetson_nano_wifi" >/dev/null 2>&1; then
    echo "Removing existing jetson_nano_wifi connection..."
    nmcli connection delete "jetson_nano_wifi"
fi

echo "Creating new hotspot connection: jetson_nano_wifi..."

# Create a shared WiFi hotspot profile using the correct IP and credentials
nmcli connection add type wifi ifname "$WIFI_IFACE" con-name "jetson_nano_wifi" ssid "jetson_nano_wifi"
nmcli connection modify "jetson_nano_wifi" 802-11-wireless.mode ap
nmcli connection modify "jetson_nano_wifi" 802-11-wireless.band bg
nmcli connection modify "jetson_nano_wifi" wifi-sec.key-mgmt wpa-psk
nmcli connection modify "jetson_nano_wifi" wifi-sec.psk "team1-falcon"
nmcli connection modify "jetson_nano_wifi" ipv4.method shared ipv4.addresses "192.168.20.1/24"

echo "Starting hotspot..."
nmcli connection up "jetson_nano_wifi"

echo "Configuring firewall to allow ESP32 traffic on port 5000..."
sudo ufw allow in on "$WIFI_IFACE" to any port 5000 proto tcp comment "Allow ESP32 server"

echo "Hotspot started successfully on $WIFI_IFACE."
echo "SSID: jetson_nano_wifi"
echo "Password: team1-falcon"
echo "Host IP: 192.168.20.1"
