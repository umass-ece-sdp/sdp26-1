# Use Python 3.11 slim image as base
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install system dependencies for OpenCV
RUN apt-get update && apt-get install -y \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libgomp1 \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the entire project
COPY . .

# Set Python path to include the software directory
ENV PYTHONPATH=/app/software/src:$PYTHONPATH

# Expose ports if needed (Tello uses UDP ports 8889, 8890, 11111)
EXPOSE 8889/udp 8890/udp 11111/udp

# Default command (can be overridden)
CMD ["python", "software/src/falcon.py"]
