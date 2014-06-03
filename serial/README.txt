The "serialTest.py" script sends a "hello" message over ttyO4.

If serial is not working run the "serialSetup.sh", which performs the commands discussed below.

http://www.armhf.com/index.php/beaglebone-black-serial-uart-device-tree-overlays-for-ubuntu-and-debian-wheezy-tty01-tty02-tty04-tty05-dtbo-files/

Copy the following .dtbo overlay files to the /lib/firmware directory and apply them after each boot with the command: echo ttyO1_armhf.com > /sys/devices/bone_capemgr*/slots

ttyO1_armhf.com-00A0.dtbo
ttyO2_armhf.com-00A0.dtbo
ttyO4_armhf.com-00A0.dtbo
ttyO5_armhf.com-00A0.dtbo
Note 1: ttyO3 does not have an RX pinout (it is tied to the TDA19988 HDMI chip)
Note 2: ttyO5 shares pins with the HDMI overlay – both cannot be active at the same time
Note 3: ttyO0 is available on J1 and does not require an overlay
