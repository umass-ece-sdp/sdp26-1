#!/usr/bin/env bash
set -e  # exit if any command fails

WIFI_AP="$1"
WIFI_CLIENT="$2"

AP_CONN="ap-$WIFI_AP"
CLIENT_CONN="client-$WIFI_CLIENT"

nmcli connection down "$AP_CONN" 2>/dev/null || true
nmcli connection delete "$AP_CONN" 2>/dev/null || true

nmcli connection down "$CLIENT_CONN" 2>/dev/null || true
nmcli connection delete "$CLIENT_CONN" 2>/dev/null || true

echo "Teardown complete."
