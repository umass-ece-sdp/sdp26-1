# Docker Setup for FALCON Drone Project

## Prerequisites
- Docker installed on your Linux system
- Docker Compose installed (or use `docker compose` with newer Docker versions)

## Building the Docker Image

### Option 1: Using Docker Compose (Recommended)
```bash
docker-compose build
```

### Option 2: Using Docker directly
```bash
docker build -t sdp26-falcon:latest .
```

## Running the Container

### Option 1: Using Docker Compose
```bash
# Start the container
docker-compose up

# Run in detached mode
docker-compose up -d

# Stop the container
docker-compose down
```

### Option 2: Using Docker directly
```bash
docker run -it --network host sdp26-falcon:latest
```

## Development Workflow

### Interactive Shell Access
```bash
# If using docker-compose
docker-compose run falcon /bin/bash

# If using docker directly
docker run -it --network host sdp26-falcon:latest /bin/bash
```

### Running Tests
```bash
docker-compose run falcon python -m pytest software/tests/
```

### Running Specific Scripts
```bash
docker-compose run falcon python software/src/falcon.py
```

## Important Notes for Drone Communication

1. **Network Mode**: The container uses `network_mode: host` to enable UDP communication with the Tello drone on ports 8889, 8890, and 11111.

2. **WiFi Connection**: Your Linux machine must be connected to the Tello drone's WiFi network. Docker will use the host's network stack.

3. **Video Streaming**: X11 display is configured for OpenCV windows. Make sure to allow X11 connections:
   ```bash
   xhost +local:docker
   ```

## Linux-Specific Setup for X11 Display

To enable OpenCV video display from the container:

```bash
# Allow Docker to access X11 display
xhost +local:docker

# Run the container (X11 is already configured in docker-compose.yml)
docker-compose up
```

To revoke access after you're done:
```bash
xhost -local:docker
```

## Troubleshooting

### Container can't connect to drone
- Ensure your computer is connected to the drone's WiFi
- Verify network_mode is set to "host"
- Check that no firewall is blocking UDP ports (8889, 8890, 11111)

### OpenCV display issues
- Run `xhost +local:docker` before starting the container
- Verify the DISPLAY environment variable is set correctly
- Check that X11 socket is mounted: `/tmp/.X11-unix`

### Permission issues with volumes
- Ensure your user has read/write permissions on mounted directories
- If needed, fix ownership: `sudo chown -R $USER:$USER ./software ./data ./results`

### Docker daemon issues
- Make sure Docker service is running: `sudo systemctl status docker`
- Add your user to docker group to run without sudo: `sudo usermod -aG docker $USER`
- Log out and back in for group changes to take effect
