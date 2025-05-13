#!/bin/bash
SCRIPT_DIR="$(dirname "$0")"
CURRENT_DIR="$(pwd)"

if [ "$SCRIPT_DIR" == "$CURRENT_DIR" ] || [ "$SCRIPT_DIR" == "." ]; then
    echo "The script is being run from its own directory."
else
    echo "The script is NOT being run from its own directory."
    exit 1
fi

./install_ros_humble.sh
source ~/.bashrc
./install_gstreamer.sh
source ~/.bashrc
./install_nav_deps.sh
source ~/.bashrc
./setup_service.sh

sudo cp enablecan.sh /usr/local/bin || exit 1
echo "$USER ALL=(ALL) NOPASSWD: /usr/local/bin/enablecan.sh" | sudo tee -a /etc/sudoers > /dev/null
sudo cp turnOnRaman.sh /usr/local/bin
sudo chmod 755 /usr/local/bin/turnOnRaman.sh

echo "${SUDO_USER:-$USER} ALL=(ALL) NOPASSWD: /usr/local/bin/turnOnRaman.sh" \
  | sudo tee /etc/sudoers.d/turnonraman >/dev/null
sudo chmod 0440 /etc/sudoers.d/turnonraman
