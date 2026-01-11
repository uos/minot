#!/bin/bash
 
apt-get install -y curl git unzip clang
apt-get update --assume-yes && apt-get install locales --assume-yes
locale-gen en_US en_US.UTF-8
export LANG=en_US.UTF-8
apt-get install software-properties-common --assume-yes

# ROS2 Jazzy
add-apt-repository universe -y
apt-get update --assume-yes
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt-get update --assume-yes && apt-get install ros-dev-tools ros-jazzy-ros-base --assume-yes

rosdep init
rosdep update

