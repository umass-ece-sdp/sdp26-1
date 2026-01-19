#!/usr/bin/env bash

set -e  # exit if any command fails

# Go to repo
echo "Starting VENV at $(date)" >&2
cd /home/falcon/sdp26-1

# Run main
echo "Starting main script at $(date)" >&2
falcon
