#!/usr/bin/env bash
set -e

# Detect the active WiFi interface
WIFI_IFACE=$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi" {print $1}' | head -n 1)

echo "Stopping and removing hotspot connection: jetson_nano_wifi..."

echo "Removing firewall rule for ESP32..."
sudo ufw delete allow in on "$WIFI_IFACE" to any port 5000 proto tcp 2>/dev/null || true

# Try bringing it down if it's currently active
nmcli connection down "jetson_nano_wifi" 2>/dev/null || true

# Remove the connection profile completely
nmcli connection delete "jetson_nano_wifi" 2>/dev/null || true

echo "Hotspot profile deleted."

if [ -n "$WIFI_IFACE" ]; then
    echo "Re-enabling normal WiFi networking on $WIFI_IFACE..."
    # A device reapply gets it back to unmanaged/managed standard state or
    # disconnecting the device makes nmcli automatically select another known connection.
    nmcli device disconnect "$WIFI_IFACE" 2>/dev/null || true
    
    # Trigger a rescan to help NetworkManager find available networks immediately
    nmcli device wifi rescan 2>/dev/null || true
    echo "WiFi successfully restored to normal client mode."
else
    echo "No WiFi interface detected to re-enable."
fi
