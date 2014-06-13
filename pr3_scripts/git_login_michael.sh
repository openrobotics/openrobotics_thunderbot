#!/bin/sh
# sets up computer so changes made are pushed to github under michael's alias
cd ~/Workspace/catkin_ws/src/athomesoftware
git remote set-url origin https://toxicseaweed@github.com/thunderbots/athomesoftware
git config --global user.name "Michael Moritsugu"
git config --global user.email "mmoritsugu@gmail.com"
git config --global color.ui true
