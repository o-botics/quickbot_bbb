#!/bin/bash
echo ttyO1_armhf.com > /sys/devices/bone_capemgr*/slots
echo ttyO2_armhf.com > /sys/devices/bone_capemgr*/slots
echo ttyO4_armhf.com > /sys/devices/bone_capemgr*/slots
echo ttyO5_armhf.com > /sys/devices/bone_capemgr*/slots
./serialSetup.py

