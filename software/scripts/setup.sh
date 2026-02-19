#!/usr/bin/env bash
set -e  # exit if any command fails

WIFI_AP="$1"
SSID_AP="$2"
PASSWORD_AP="$3"
AP_CONN="ap-$WIFI_AP"
WIFI_CLIENT="$4"
SSID_CLIENT="$5"
PASSWORD_CLIENT="$6"
CLIENT_CONN="client-$WIFI_CLIENT"

echo "=== STARTING WIFI ==="

# Delete old connections
nmcli connection delete "$AP_CONN" 2>/dev/null || true
nmcli connection delete "$CLIENT_CONN" 2>/dev/null || true

# Create AP connection
nmcli connection add type wifi \
    ifname "$WIFI_AP" \
    con-name "$AP_CONN" \
    autoconnect no \
    ssid "$SSID_AP"
nmcli connection modify "$AP_CONN" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$PASSWORD_AP" \
    ipv4.method manual \
    ipv4.addresses 192.168.50.1/24 \
    ipv6.method ignore
nmcli connection up "$AP_CONN"

# Connect to drone
nmcli device wifi connect "$SSID_CLIENT" \
    password "$PASSWORD_CLIENT" \
    ifname "$WIFI_CLIENT" \
    name "$CLIENT_CONN"
MAX_WAIT=30
COUNT=0
while true; do
    STATE=$(nmcli -t -f GENERAL.STATE device show "$WIFI_CLIENT" 2>/dev/null | cut -d: -f2 | awk '{print $1}')
    STATE=${STATE:-0}
    if [[ "$STATE" -eq 100 ]]; then
        echo "Connection established."
        break
    fi
    if [ "$COUNT" -ge "$MAX_WAIT" ]; then
        echo "ERROR: Connection timed out."
        exit 1
    fi
    sleep 1
    COUNT=$((COUNT+1))
done
