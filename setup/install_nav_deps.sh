#!/bin/bash

echo "Downloading ZED SDK ..."
cd /tmp
if [ "$(uname -m)" == "aarch64" ]; then
curl -L https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Tegra_L4T36.4_v4.2.2.zstd.run -o zed_sdk_installer.run
else
curl -L https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu22 -o zed_sdk_installer.run
fi
chmod +x zed_sdk_installer.run
./zed_sdk_installer.run -- silent
echo "Finished downloading ZED SDK ..."

echo "Building Kindr ..."
cd /tmp
git clone https://github.com/ANYbotics/kindr.git
cd kindr/
mkdir build
cd build
cmake .. -DUSE_CMAKE=true
sudo make install
echo "Finished building Kindr ..."

sudo mkdir -p /usr/local/cprt
sudo chown $USER:$USER /usr/local/cprt