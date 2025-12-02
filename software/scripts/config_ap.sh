#!/usr/bin/env bash

set -e  # exit if any command fails

# Make sure the correct ip address is assigned to the interface
ip addr del 192.168.10.1/24 dev wlx200cc83f101b
ip addr add 192.168.20.1/24 dev wlx200cc83f101b
systemctl restart dnsmasq

# Ensure WiFi services are started and enabled
echo "Starting WiFi services at $(date)" >&2
systemctl start dnsmasq
systemctl start hostapd