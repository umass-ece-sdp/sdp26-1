#!/usr/bin/env bash

set -e  # exit if any command fails

# Ensure WiFi services are started and enabled
echo "Starting WiFi services at $(date)" >&2
systemctl start dnsmasq
systemctl start hostapd

# Go to repo
echo "Starting VENV at $(date)" >&2
cd /home/falcon/sdp26-1

# Activate venv
source .venv/bin/activate   # adjust if the name is different

# Run main
echo "Starting main script at $(date)" >&2
falcon
