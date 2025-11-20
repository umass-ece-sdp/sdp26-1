#!/usr/bin/env bash
# Usage: ./connect_wifi.sh <IFACE> <SSID> <PASSWORD>
IFACE="$1"
SSID="$2"
PASSWORD="$3"

if [ -z "$IFACE" ] || [ -z "$SSID" ]; then
  echo "Usage: $0 <IFACE> <SSID> <PASSWORD>"
  exit 2
fi

echo "Bringing interface $IFACE up..."
sudo ip link set "$IFACE" up || echo "ip link failed (maybe already up)"

echo "Scanning for SSID '$SSID' on interface $IFACE..."
# list visible networks on that interface
nmcli -f SSID,SIGNAL,CHAN device wifi list ifname "$IFACE"

echo "Trying to connect to SSID $SSID..."
sudo nmcli device wifi connect "$SSID" password "$PASSWORD" ifname "$IFACE"

RC=$?
if [ $RC -eq 0 ]; then
  echo "Connected to $SSID on $IFACE"
  nmcli -g GENERAL.STATE device show "$IFACE"
  
  # Add explicit route to ensure Tello traffic goes through correct interface
  echo "Adding route for Tello network through $IFACE..."
  sudo ip route add 192.168.10.0/24 dev "$IFACE" 2>/dev/null || echo "Route already exists or failed to add"
else
  echo "Failed to connect (exit $RC)."
  nmcli --terse --fields STATE,STATE_ACTIVE device show "$IFACE"
fi

exit $RC

