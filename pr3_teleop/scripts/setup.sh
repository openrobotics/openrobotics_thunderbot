#!/bin/bash

sudo hciconfig hci0 reset
sudo sixpair
sixad --start
