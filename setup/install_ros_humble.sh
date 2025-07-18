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
ninja-build \
ros-humble-rmw-cyclonedds-cpp \
ros-humble-rosbridge-server \
python3-usb

source /opt/ros/humble/setup.bash

if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  echo "ROS 2 sourced in bashrc"
fi

pip3 install black
pip3 install pylint
pip3 install pyserial
pip3 install adafruit-circuitpython-lis3dh
pip3 install adafruit-circuitpython-pca9685
pip3 install adafruit-circuitpython-motorkit 
pip3 install Adafruit-Blinka
pip3 install adafruit-circuitpython-bme280
pip3 install adafruit-circuitpython-ens160
pip3 install adafruit-circuitpython-busdevice
pip3 install adafruit-circuitpython-register
pip3 install adafruit-circuitpython-neopixel-spi
pip3 install pyubx2
pip3 install numpy
pip3 install pyusb==1.2.1

sudo rosdep init
rosdep update

sudo usermod -a -G dialout $USER

echo "ROS2 humble install complete!"