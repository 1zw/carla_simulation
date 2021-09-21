#!/bin/sh

set -e

# Record the time this script starts
date

# Get the full dir name of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Keep updating the existing sudo time stamp
sudo -v
while true; do sudo -n true; sleep 120; kill -0 "$$" || exit; done 2>/dev/null &

echo "\e[100m Install CARLA 0.9.11 \e[0m"
cd
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.11.tar.gz
tar -xf CARLA_0.9.11.tar.gz

echo "\e[100m Install ROS Foxy Fitzroy \e[0m"
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-foxy-desktop

echo "\e[100m Install Additional Dependencies \e[0m"
sudo apt install -y \
            python3-numpy \
            python3-pygame \
            python3-pip \
            python3-colcon-common-extensions \
            python3-rosdep \
            python3-opencv \
            ros-foxy-image-pipeline
sudo rosdep init
rosdep update
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
. /opt/ros/foxy/setup.bash

echo "\e[100m Install CARLA ROS Bridge \e[0m"
cd ~/carla-ros-bridge/src
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git ros-bridge
cd ros-bridge
git checkout tags/0.9.11
cd ~/carla-ros-bridge
rosdep install --from-paths src --ignore-src -r

echo "\e[100m All done! \e[0m"

#record the time this script ends
date