# FALCON – Filming Autonomous Learning and Cinematic Optic Navigator

Autonomous DJI Tello color-tracking platform with a wearable glove controller. A Jetson host (or Linux laptop) runs vision and flight control while an ESP32-S3 glove sends discrete range commands over Wi-Fi. The system locks onto a target color, keeps the subject centered and at a commanded distance, searches when lost, and lands safely on failure.

---

## Repository Map

- [software/src/main.py](software/src/main.py): Entry point; configures the access point IP, starts the glove server, and launches drone tracking threads.
- [software/lib/falcon_vision.py](software/lib/falcon_vision.py): Vision + control loop (HSV color mask primary; YOLO assist available but off), distance mapping from glove instructions, safety land/search logic.
- [software/lib/falcon.py](software/lib/falcon.py): DJI Tello wrapper that auto-connects to the drone Wi-Fi using a provided interface/SSID.
- [hardware/firmware](hardware/firmware): ESP32-S3 glove firmware, TCP glove server, and PlatformIO config.
- [software/scripts](software/scripts): Network helper scripts (AP config, Tello Wi-Fi connect) and startup helper for the host.
- [docs](docs): Design docs placeholders (currently empty); fill as deliverables are produced.

---

## System Overview

- **Vision/Control (host):** Tracks a target color (#a61919 by default) in the Tello video feed, computes yaw/altitude/forward corrections, and commands the drone. Falls back to search spins when target is lost; lands after timeout or manual quit (press `q`). See [software/lib/falcon_vision.py](software/lib/falcon_vision.py).
- **Glove (ESP32-S3 DevKitC-1):** Four three-state switches encode a 4-char string (LR, FB, heading, altitude). Host maps specific patterns to target distances (e.g., `0000` → 50 cm, `1000` → 25 cm, `2000` → 100 cm). See [hardware/firmware/main/glove.cpp](hardware/firmware/main/glove.cpp) and mapping in [software/lib/falcon_vision.py](software/lib/falcon_vision.py).
- **Networking:** Host runs a TCP server on 192.168.20.1:5000 for the glove. The host also connects to the Tello SSID (default TELLO-AA7B55) via a dedicated interface before starting control. AP and Tello routes are configured with helper scripts under [software/scripts](software/scripts).

---

## Hardware

- DJI Tello drone.
- ESP32-S3-DevKitC-1 (Arduino/PlatformIO) running glove client.
- Four 3-state switches wired to pins (LR: GPIO4/5, FB: 6/7, Heading: 15/16, Altitude: 17/18). See [hardware/firmware/main/glove.cpp](hardware/firmware/main/glove.cpp).
- Host computer (Jetson Nano or Linux laptop) with Wi-Fi interfaces for both AP (glove) and Tello.

---

## Firmware (Glove) Setup

1. Install PlatformIO.
2. From [hardware/firmware](hardware/firmware):
   - Build: `platformio run -e client`
   - Flash: `platformio run -e client -t upload`
   - Serial monitor: `platformio device monitor -e client`
3. Network config lives in [hardware/firmware/main/wifi_client.h](hardware/firmware/main/wifi_client.h) (SSID `jetson_nano_wifi`, password `team1-falcon`, host `192.168.20.1`, port `5000`). Update if your AP differs.
4. Glove client sends a 4-char command every second and expects `ACK` from the host server; reconnects on timeout. See [hardware/firmware/main/wifi_client.cpp](hardware/firmware/main/wifi_client.cpp) and [hardware/firmware/server.py](hardware/firmware/server.py).

---

## Software Prereqs

- Python 3.10+
- Dependencies: djitellopy, opencv-python, numpy, ultralytics (declared in [pyproject.toml](pyproject.toml)).
- YOLO model file already present at [software/lib/yolo11n.pt](software/lib/yolo11n.pt); YOLO path is set in [software/lib/falcon_vision.py](software/lib/falcon_vision.py).

Install (recommended in a virtualenv):

```bash
pip install -e .
```

---

## Network Setup (Host)

1. **Access Point for glove (192.168.20.1):**
   - Interface default: `wlx200cc83f101b` in [software/scripts/config_ap.sh](software/scripts/config_ap.sh).
   - Run: `sudo bash software/scripts/config_ap.sh`
   - Starts dnsmasq/hostapd and assigns 192.168.20.1/24.
2. **Connect to Tello:**
   - Interface default: `wlx90de80899a92`, SSID `TELLO-AA7B55`, empty password in [software/lib/falcon.py](software/lib/falcon.py).
   - Script usage: `sudo bash software/scripts/connection_client.sh <iface> <ssid> <password>`
   - Adds routes for 192.168.10.0/24 so Tello traffic uses that interface.

Adjust interface names/SSID if your hardware differs.

---

## Running the System

1. Power on the ESP32 glove; ensure it joins the AP (`jetson_nano_wifi`).
2. On the host, activate your Python env and run:

```bash
falcon
# or
python -m software.src.main
```

This will:
- Configure the AP IP (step 1 above).
- Start the glove TCP server and wait for a glove connection.
- Connect to the Tello, start the video stream, take off, and begin color tracking.

Alternate: On the Jetson, [software/scripts/automate_startup.sh](software/scripts/automate_startup.sh) cd’s to the repo and runs `falcon` (assumes env already set up).

---

## Glove Instruction Mapping (distance presets)

- `0000` → target distance 50 cm
- `1000` → target distance 25 cm
- `2000` → target distance 100 cm

Other patterns are logged but ignored for distance changes. The vision loop re-computes target area each time a mapped pattern arrives.

---

## Behavior and Safety

- Hover/search: If the target disappears, the drone yaws to search; after a timeout it lands.
- Manual exit: Press `q` in the vision window to land and stop.
- Connection failures: Tello Wi-Fi connect failures abort startup; glove disconnects clear the shared state in [software/lib/variables.py](software/lib/variables.py).
- Stream cleanup: On any exit path, video stream is stopped and RC commands zeroed before ending.

---

## Development Notes

- Vision currently uses HSV color masking; YOLO detection code is present but commented out. Tweak target color, tolerances, and speeds near the top of [software/lib/falcon_vision.py](software/lib/falcon_vision.py).
- Empty doc folders contain `.gitkeep`; populate [docs/design_specs](docs/design_specs) and [docs/final_report](docs/final_report) as artifacts are produced.
- No automated tests yet; add unit tests under `software/tests/` and leverage PlatformIO monitors for firmware.

---

## Quick Script Reference

- AP setup: [software/scripts/config_ap.sh](software/scripts/config_ap.sh)
- Connect to Tello Wi-Fi: [software/scripts/connection_client.sh](software/scripts/connection_client.sh)
- Host startup helper: [software/scripts/automate_startup.sh](software/scripts/automate_startup.sh)
- Glove TCP server (Python): [hardware/firmware/server.py](hardware/firmware/server.py)
