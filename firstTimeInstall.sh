#!/bin/bash

locale  # check for UTF-8

set -e

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade -y

sudo apt install -y \
ros-humble-desktop \
ros-dev-tools \
python3-dev \
python3-pip \
clang-format \
clang-tidy \
ninja-build

source /opt/ros/humble/setup.bash

if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  echo "ROS 2 sourced in bashrc"
fi

pip3 install black
pip3 install pylint
pip3 install pyubx2

sudo rosdep init
rosdep update

sudo usermod -a -G dialout $USER

echo "ROS2 humble install complete!"

echo "Starting Gstreamer upgrade..."

sudo apt-get update && sudo apt-get install -y zlib1g-dev libffi-dev libssl-dev python3-dev python3-pip flex bison libglib2.0-dev libmount-dev
python3 -m pip install --upgrade pip
pip3 install meson
export PATH=$PATH:~/.local/bin
if ! grep -q "export PATH=\$PATH:~/.local/bin" ~/.bashrc; then
  echo "export PATH=\$PATH:~/.local/bin" >> ~/.bashrc
  echo "meson exported in bashrc"
fi

cd ~
git clone https://gitlab.freedesktop.org/gstreamer/gstreamer.git
cd gstreamer
git checkout 1.24
meson setup builddir
meson compile -C builddir

export GSTREAMER_DIR=$PWD
./gst-env.py --only-environment > setupGstreamer.sh
sed -i '/PWD/d' setupGstreamer.sh
sudo chmod +x setupGstreamer.sh
source setupGstreamer.sh
if ! grep -q "source $GSTREAMER_DIR/setupGstreamer.sh" ~/.bashrc; then
  echo "source $GSTREAMER_DIR/setupGstreamer.sh" >> ~/.bashrc
  echo "Gstreamer sourced in bashrc"
fi

cd /tmp
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
. "$HOME/.cargo/env"
git clone https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs.git

# Webrtc
cd gst-plugins-rs/net/webrtc
cargo build --release --target-dir build
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.0/install.sh | bash
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
nvm install 22
node -v
npm -v
cd gstwebrtc-api/
npm install
npm run build
cd ../..
cp -r webrtc $GSTREAMER_DIR
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$GSTREAMER_DIR/webrtc/build/release

# Rtp and congestion controll
cd /tmp/gst-plugins-rs/net/rtp
cargo build --release --target-dir build
cd ..
cp -r rtp $GSTREAMER_DIR
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$GSTREAMER_DIR/rtp/build/release

export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:/usr/lib/aarch64-linux-gnu/gstreamer-1.0/deepstream:/usr/lib/aarch64-linux-gnu/gstreamer-1.0/

echo "export GST_PLUGIN_PATH=$GST_PLUGIN_PATH" >> $GSTREAMER_DIR/setupGstreamer.sh

echo "Finished building GStreamer"

source ~/.bashrc