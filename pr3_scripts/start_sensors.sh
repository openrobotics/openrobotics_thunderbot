#!/bin/bash
#Thunderbots@Home Software Team, University of British Columbia
# Author: Devon Ash
# Created: 21/09/13
# Initializes sensor topics for the robot
# Must be called from a directory that is inside the ROS_PACKAGE_PATH.

## Path variables
SENSORS_LAUNCH_PATH="../sensors/launch/start_sensors.launch"

## Configure the GSCAM variables for using integrated webcam
export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb ! ffmpegcolorspace" 

## Launch integrated cam
## Launch USB cam
## Launch Microphone
## Launch Speakers
## Launch Kinect
## Launch Lidar

## Debugging things, so commented these out
## echo "Starting sensors for robot..."
## roslaunch $SENSORS_LAUNCH_PATH

