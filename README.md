# QuickBot_BBB
This is the code that runs on the BeagleBone Black to control the QuickBot.

## Overview
Essentially this code establishes socket (UDP) connection with another device (BASE) and waits for commands. The commands are either of the form of directives or queries. An example directive is setting the PWM values of the motors. An example query is getting IR sensor values.

## Installation
Clone the repo into home directory:

	cd ~
	git clone https://bitbucket.org/rowoflo/quickbot_bbb.git

## Running
Check IP address of BASE and ROBOT (run command on both systems and look for IP address):

	ifconfig

Example output from BBB:

	ra0       Link encap:Ethernet  HWaddr 00:0C:43:00:14:F8
	          inet addr:192.168.1.101  Bcast:192.168.1.255  Mask:255.255.255.0
	          inet6 addr: fe80::20c:43ff:fe00:14f8/64 Scope:Link
	          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
	          RX packets:315687 errors:1113 dropped:1 overruns:0 frame:0
	          TX packets:12321 errors:0 dropped:0 overruns:0 carrier:0
	          collisions:0 txqueuelen:1000
	          RX bytes:66840454 (63.7 MiB)  TX bytes:1878384 (1.7 MiB)

Here the IP address for the robot is 192.168.1.101. Let's assume the IP address for the BASE is 192.168.1.100.

Change into working directory:

	cd ~/quickbot_bbb

Launch QuickBotRun python script using IP addresses of BASE and ROBOT:

	./QuickBotRun.py 192.168.1.100 192.168.1.101

## Command Set

* Check that the QuickBot is up and running:
  * Command

		"$CHECK*\n"

  * Response

		"Hello from QuickBot\n"


* Get PWM values:
  * Command

		"$PWM?*\n"

  * Example response

		"[50, -50]\n"


* Set PWM values:
  * Command

		"$PWM=[-100,100]*\n"


* Get IR values:
  * Command

		"$IRVAL?*\n"

  * Example response

		"[800, 810, 820, 830, 840]\n"


* Get encoder position:
  * Command

		"$ENVAL?*\n"

  * Example response

		"[200, -200]\n"


* Get encoder velocity (tick velocity -- 16 ticks per rotation):
  * Command

		"$ENVEL?*\n"

  * Example response

		"[20.0, -20.0]\n"


* Reset encoder position to zero:
  * Command

		"$RESET*\n"


* End program
  * Command:

 		"$END*\n"

