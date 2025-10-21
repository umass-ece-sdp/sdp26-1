# Docker Setup for FALCON Drone Project

## Prerequisites
- Docker Desktop installed on your machine
- Docker Compose (included with Docker Desktop)

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

2. **WiFi Connection**: Your host machine must be connected to the Tello drone's WiFi network. Docker will use the host's network stack.

3. **Video Streaming**: If you need to display video from the drone, you may need to configure X11 forwarding (Linux/Mac) or use alternative display methods on Windows.

## Windows-Specific Considerations

On Windows, Docker runs in a VM, which can complicate UDP networking. Consider:
- Running the Python code directly on Windows for drone control
- Using WSL2 with Docker for better networking
- Or using the container for development/testing with mock data

## Troubleshooting

### Container can't connect to drone
- Ensure your computer is connected to the drone's WiFi
- Verify network_mode is set to "host"
- Check that no firewall is blocking UDP ports

### OpenCV display issues
- On Windows, OpenCV windows won't display from Docker containers
- Consider saving images/videos to mounted volumes instead
- Or run the code natively on Windows for GUI features

### Permission issues with volumes
- On Windows, ensure Docker has access to shared drives
- Check Docker Desktop settings > Resources > File Sharing
