#!/usr/bin/env bash

set -e  # exit if any command fails

# Ensure WiFi services are started and enabled
systemctl start dnsmasq
systemctl start hostapd

# Go to repo
cd /home/falcon/sdp26-1

# Activate venv
source .venv/bin/activate   # adjust if the name is different

# Run main
falcon