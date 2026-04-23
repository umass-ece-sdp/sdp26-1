# SDP26 Team 1 | FALCON – Filming Autonomous Learning and Cinematic Optic Navigator

Autonomous DJI Tello target-tracking platform with a wearable glove controller. A Jetson host (or Linux laptop) runs vision and flight control while an ESP32-S3 glove sends discrete range commands over Wi-Fi. The system locks onto a target, keeps the subject centered and at a commanded distance, searches when lost, and lands safely on failure.

---

## Repository Map

- [software/src/main.py](software/src/main.py): Entry point - connects to Tello drone and ESP32 glove AP, starts the glove client and drone tracking threads
- [software/lib/falcon_vision.py](software/lib/falcon_vision.py): Vision + control loop, distance mapping from glove instructions, safety land/search logic
- [software/lib/falcon.py](software/lib/falcon.py): DJI Tello wrapper that auto-connects to the drone Wi-Fi using a provided interface/SSID
- [hardware/firmware](hardware/firmware): ESP32-S3 glove firmware with WiFi AP and TCP server, plus Python client for base station
- [software/scripts](software/scripts): Network helper scripts (WiFi AP setup for Tello and glove connections) and startup helper for the host
- [docs](docs): Design docs placeholders (currently empty); fill as deliverables are produced

---

## System Overview

- **Vision/Control (host):** Tracks a target in the Tello video feed, computes yaw/altitude/forward corrections, and commands the drone. Falls back to search spins when target is lost; lands after timeout or manual quit (press `q`). See [software/lib/falcon_vision.py](software/lib/falcon_vision.py).
- **Glove (ESP32-S3 DevKitC-1):** Four three-state switches encode a 4-char string (LR, FB, heading, altitude). Reads IMU and stretch sensors, computes hand velocity, packages data into 6-float packets, and sends to base station. Also creates the WiFi AP. See [hardware/firmware/main/glove.cpp](hardware/firmware/main/glove.cpp) and [hardware/firmware/main/wifi_server.cpp](hardware/firmware/main/wifi_server.cpp).
- **Networking:** The **ESP32 glove creates a WiFi AP** named **"FALCON-Glove"** (password: `team1-falcon`) on 192.168.4.0/24 and runs a **TCP server at 192.168.4.1:5000**. The **base station (host) connects as a client** to this AP and connects to the glove's TCP server. Separately, the base station connects to the Tello drone on its own WiFi SSID using a second interface. See [hardware/firmware/main/wifi_server.h](hardware/firmware/main/wifi_server.h) and [hardware/firmware/glove_client.py](hardware/firmware/glove_client.py) for configuration.

---

## Hardware

- DJI Tello drone.
- ESP32-S3-DevKitC-1 (Arduino/PlatformIO) running glove sensors and WiFi AP + TCP server.
- Four 3-state switches wired to pins (LR: GPIO4/5, FB: 6/7, Heading: 15/16, Altitude: 17/18). See [hardware/firmware/main/glove.cpp](hardware/firmware/main/glove.cpp).
- Host computer (Jetson Nano, Linux laptop, or Windows machine) with **two WiFi interfaces**: one for Tello drone, one for glove AP.

---

## Firmware (Glove) Setup

1. Install PlatformIO.
2. From [hardware/firmware](hardware/firmware):
   - Build: `platformio run -e client`
   - Flash: `platformio run -e client -t upload`
   - Serial monitor: `platformio device monitor -e client`
3. WiFi/Server config lives in [hardware/firmware/main/wifi_server.h](hardware/firmware/main/wifi_server.h):
   - **AP_SSID:** `FALCON-Glove` (the AP name the glove creates)
   - **AP_PASS:** `team1-falcon` (glove AP password)
   - **AP IP:** `192.168.4.1/24` (glove runs TCP server here)
   - **PORT:** `5000` (TCP server port on glove)
4. Glove reads sensors, creates a 6-float packet (fingers[4], speed, distance) every ~90ms, and sends it to any connected base station client. See [hardware/firmware/main/wifi_server.cpp](hardware/firmware/main/wifi_server.cpp).

---

## Software Prereqs

- Python 3.10+
- Dependencies: djitellopy, opencv-python, numpy, ultralytics (declared in [pyproject.toml](pyproject.toml)).

Install (recommended in a virtualenv):

```bash
pip install -e .
```

---

## Network Setup (Host / Base Station)

**Architecture:** Glove (ESP32) is the TCP server; base station (host) is the client.

### Prerequisites

- **Glove powered on first** — It creates AP "FALCON-Glove" and starts TCP server at 192.168.4.1:5000
- **Base station has two WiFi interfaces:**
  - **Interface 1:** Connects to Tello drone SSID (e.g., "TELLO-FE046A")
  - **Interface 2:** Connects to glove AP "FALCON-Glove"

### Automatic Setup (Linux with 2 interfaces)

```bash
sudo bash software/scripts/setup_wifi.sh TELLO-FE046A "" FALCON-Glove team1-falcon
```

This script:
1. Connects Interface 1 to Tello SSID
2. Connects Interface 2 to glove AP "FALCON-Glove"
3. Verifies connectivity to both networks

### Manual Setup (Windows or Single Interface)

**For Tello:**
1. Go to **Settings → Network & Internet → WiFi**
2. Connect to **TELLO-FE046A** (empty password)

**For Glove:**
1. Go to **Settings → Network & Internet → WiFi**
2. Connect to **FALCON-Glove** (password: `team1-falcon`)
3. You should get assigned an IP like 192.168.4.x

**Verify both connections:**
```bash
ping 192.168.1x.x.x.x     # Tello IP
ping 192.168.4.1          # Glove server
```

---

## Running the System

### Startup Sequence

1. **Power on the ESP32 glove first:**
   - Creates WiFi AP "FALCON-Glove"
   - Starts TCP server at 192.168.4.1:5000
   - Waits for base station client connection

2. **On the base station, ensure both WiFi connections are active:**
   - Connected to Tello SSID
   - Connected to "FALCON-Glove" AP

3. **Run the main program:**

   ```bash
   python -m software.src.main
   ```

   This will:
   - Connect to the Tello drone and request SDK mode
   - Connect to the glove's TCP server at 192.168.4.1:5000
   - Wait for glove connection (serial monitor should show "[SERVER] Base station connected from...")
   - Take off and begin tracking the target
   - Respond to glove sensor input and gesture commands

### Expected Output

**Glove Serial Monitor:**
```
[AP] Access Point created successfully!
[AP] SSID: FALCON-Glove
[AP] IP Address: 192.168.4.1
[SERVER] Base station connected from: 192.168.4.x
[SERVER] Sent packet (24 bytes)
```

**Base Station Terminal:**
```
[CLIENT] Connecting to glove server at 192.168.4.1:5000...
[CLIENT] Connected to glove server!
Binding complete
Connected with 192.168.4.1: 5000
```


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
