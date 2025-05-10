#!/bin/bash
SCRIPT_DIR="$(dirname "$0")"
CURRENT_DIR="$(pwd)"
CURRENT_USER=$(whoami)
VERSION="science-1.0.0"

# Check if the script is being run from its own directory
if [ "$SCRIPT_DIR" == "$CURRENT_DIR" ] || [ "$SCRIPT_DIR" == "." ]; then
    echo "The script is being run from its own directory."
else
    echo "The script is NOT being run from its own directory."
    exit 1
fi

# Install GStreamer dependencies
sudo apt-get install -y \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-bad1.0-dev \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        gstreamer1.0-tools \
        gstreamer1.0-x \
        gstreamer1.0-gl \
        gstreamer1.0-gtk3 \
        gstreamer1.0-qt5 \
        libx264-dev \
        libjpeg-dev \
        gstreamer1.0-libcamera

sudo bash -c 'echo "dtoverlay=pwm-2chan" >> /boot/firmware/config.txt'

# Install Docker
if ! command -v docker &> /dev/null; then
    echo "Docker not found, installing Docker..."

    # Update apt package list
    sudo apt-get update

    sudo apt-get install -y docker.io

    # Verify Docker installation
    sudo systemctl enable --now docker
    sudo usermod -aG docker $CURRENT_USER
    echo "Docker installed and configured."
else
    echo "Docker is already installed."
fi

# Build the Docker image for the project
echo "Pulling Docker image..."
docker pull cprtsoftware/cprt_rover_24:${VERSION}

# Setup science service
sudo cp start_science.service /etc/systemd/system/start_science.service
sudo sed -i "s|User=%i|User=${CURRENT_USER}|" /etc/systemd/system/start_science.service
sudo chmod 644 /etc/systemd/system/start_science.service
sudo ln -s $PWD/start_science.sh /usr/local/bin/start_science.sh

# Reload systemd, enable, and start the service
sudo systemctl daemon-reload
sudo systemctl enable start_science.service
sudo systemctl start start_science.service

echo "Setup completed successfully."
