#!/bin/bash
sudo apt update
sudo apt install -y git
sudo apt-get install -y ros-noetic-cv-bridge
sudo apt-get install -y ros-noetic-pcl-conversions
sudo apt-get install -y ros-noetic-pcl-ros
sudo apt-get install -y ros-noetic-eigen-conversions
sudo apt-get remove -y libpcl-dev
sudo apt-get install -y libpcl-dev
sudo apt-get install -y libusb-dev
sudo apt-get install -y ros-noetic-pcl-ros
