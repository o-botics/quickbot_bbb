#!/bin/bash
cp ~/quickbot_bbb/serial/ttyO1_armhf.com-00A0.dtbo /lib/firmware
cp ~/quickbot_bbb/serial/ttyO2_armhf.com-00A0.dtbo /lib/firmware
cp ~/quickbot_bbb/serial/ttyO4_armhf.com-00A0.dtbo /lib/firmware
cp ~/quickbot_bbb/serial/ttyO5_armhf.com-00A0.dtbo /lib/firmware

echo ttyO1_armhf.com > /sys/devices/bone_capemgr*/slots
echo ttyO2_armhf.com > /sys/devices/bone_capemgr*/slots
echo ttyO4_armhf.com > /sys/devices/bone_capemgr*/slots
echo ttyO5_armhf.com > /sys/devices/bone_capemgr*/slots
~/quickbot_bbb/serial/serialSetup.py

