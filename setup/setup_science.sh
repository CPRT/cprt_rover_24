#!/bin/bash
SCRIPT_DIR="$(dirname "$0")"
CURRENT_DIR="$(pwd)"
CURRENT_USER=$(whoami)

if [ "$SCRIPT_DIR" == "$CURRENT_DIR" ] || [ "$SCRIPT_DIR" == "." ]; then
    echo "The script is being run from its own directory."
else
    echo "The script is NOT being run from its own directory."
    exit 1
fi

./install_ros_humble.sh
source ~/.bashrc
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
        gstreamer1.0-qt5

# Setup science service
sudo cp start_science.service /etc/systemd/system/start_science.service
sudo sed -i "s|User=%i|User=${CURRENT_USER}|" /etc/systemd/system/start_science.service
sudo chmod 644 /etc/systemd/system/start_science.service
sudo ln -s $PWD/start_science.sh /usr/local/bin/start_science.sh

sudo tee /opt/ros/humble/cprt_setup.bash > /dev/null << EOF
#!/bin/bash
source $PWD/install/setup.bash
EOF

sudo systemctl daemon-reload
sudo systemctl enable start_science.service
sudo systemctl start start_science.service
