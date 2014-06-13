#!/bin/sh
# Author: Devon Ash
# Maintainer: noobaca2@gmail.com

# This is a script of the readme found on the @home wiki.
sudo hciconfig hci0 reset
sudo sixpair
echo "Please unplug the USB controller from the USB or the USB from the computer and then press any key to continue"
read -n 1 -s
echo "Please open a new terminal and run: roslaunch talos_teleop teleop.launch"
sixad --start 


