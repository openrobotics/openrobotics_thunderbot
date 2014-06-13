#!/bin/sh
# sets up computer so changes made are pushed to github under michael's alias
cd ~/Workspace/catkin_ws/src/athomesoftware
git remote set-url origin https://TheDash@github.com/thunderbots/athomesoftware
git config --global user.name "Devon Ash"
git config --global user.email "noobaca@gmail.com"
git config --global color.ui true
