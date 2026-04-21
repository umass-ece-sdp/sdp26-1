#!/usr/bin/env bash
set -e

# Detect all available WiFi interfaces
mapfile -t WIFI_IFACES < <(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi" {print $1}')

echo "Stopping and removing hotspot connection: jetson_nano_wifi..."

echo "Removing firewall rule for ESP32..."
for WIFI_IFACE in "${WIFI_IFACES[@]}"; do
    sudo ufw delete allow in on "$WIFI_IFACE" to any port 5000 proto tcp 2>/dev/null || true
done

# Try bringing it down if it's currently active
nmcli connection down "jetson_nano_wifi" 2>/dev/null || true

# Remove the connection profile completely
nmcli connection delete "jetson_nano_wifi" 2>/dev/null || true

echo "Hotspot profile deleted."

if [ "${#WIFI_IFACES[@]}" -gt 0 ]; then
    for WIFI_IFACE in "${WIFI_IFACES[@]}"; do
        echo "Re-enabling normal WiFi networking on $WIFI_IFACE..."
        # Disconnecting the device allows NetworkManager to reselect normal client profiles.
        nmcli device disconnect "$WIFI_IFACE" 2>/dev/null || true

        # Trigger a rescan to help NetworkManager find available networks immediately.
        nmcli device wifi rescan ifname "$WIFI_IFACE" 2>/dev/null || true
    done
    echo "WiFi interface(s) successfully restored to normal client mode."
else
    echo "No WiFi interface detected to re-enable."
fi
