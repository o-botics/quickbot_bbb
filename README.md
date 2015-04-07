# QuickBot BeagleBone Black Repo
This is the code that runs on the BeagleBone Black to control the QuickBot.

## Overview
Essentially this code establishes socket (UDP) connection with another device
(BASE) and waits for commands. The commands are either of the form of
directives or queries. An example directive is setting the PWM values of the
motors. An example query is getting IR sensor values.

## Installation
Clone the repo into home directory:

	>> cd ~
	>> git clone https://github.com/o-botics/quickbot_bbb.git

## Running
Check IP address of BASE and ROBOT (run command on both systems and look for IP
address):

	>> ifconfig

Example output from BBB:

	ra0       Link encap:Ethernet  HWaddr 00:0C:43:00:14:F8
	          inet addr:192.168.1.101  Bcast:192.168.1.255  Mask:255.255.255.0
	          inet6 addr: fe80::20c:43ff:fe00:14f8/64 Scope:Link
	          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
	          RX packets:315687 errors:1113 dropped:1 overruns:0 frame:0
	          TX packets:12321 errors:0 dropped:0 overruns:0 carrier:0
	          collisions:0 txqueuelen:1000
	          RX bytes:66840454 (63.7 MiB)  TX bytes:1878384 (1.7 MiB)

Here the IP address for the robot is **192.168.1.101**. Let's assume the IP address
for the BASE is **192.168.1.100**.

Change into working directory:

	>> cd ~/quickbot_bbb

Quick run with given ips:

    >> python run.py -i 192.168.1.100 -r 192.168.1.101

Use the help `--help` command to display the run command syntax.
For example

    >> python run.py --help
    usage: run.py [-h] [--ip IP] [--rip RIP] [--rtype RTYPE]

    optional arguments:
      -h, --help            show this help message and exit
      --ip IP, -i IP        Computer ip (base ip)
      --rip RIP, -r RIP     BBB ip (robot ip)
      --rtype RTYPE, -t RTYPE
                            Type of robot (quick|ultra)

The QuickBot is now ready and waiting for commands on what to do.
These commands are special strings that sent over the UDP connection (see the full list of commands below).

## Send Commands From Python Example
For example to send a command to the QuickBot from your local machine using Python do the following from the command line on your local machine.

Open python interpreter:

    >> python

Set up a socket to communicate with in Python

    >>> import socket
    >>> LOCAL_IP = "192.168.1.100" # Computer IP address (change to correct value)
    >>> QB_IP = "192.168.1.101" # QuickBot IP address (change to correct value)
    >>> PORT = 5005
    >>> sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    >>> sock.setblocking(False)
    >>> sock.bind((LOCAL_IP, PORT))

Send a command to check if the QuickBot is ready

    >>> sock.sendto("$CHECK*\n", (QB_IP, PORT))

You should see `$CHECK*` appear in the QuickBot terminal where `python run.py` is running.
To spin the wheels at 100% do

    >>> sock.sendto("$PWM=100,100*\n", (QB_IP, PORT))

To stop the wheels do

    >>> sock.sendto("$PWM=100,100*\n", (QB_IP, PORT))

To check the IR values do

    >>> sock.sendto("$IRVAL?*\n", (QB_IP, PORT))

The values will appear in the QuickBot terminal, to read them from the python on your local machine do

    >>> sock.recv(1024)
    '[0.0, 0.0, 0.0, 740.0, 0.0]\n'

## Command Set

* Check that the QuickBot is up and running:
  * Command

		`"$CHECK*\n"`

  * Response:

		`'Hello from QuickBot\n'`

* Set PWM values (e.g. to left wheel 70% backwards, right wheel 90% forward):
  * Command

        `"$PWM=-70,90*\n"`

* Get PWM values:
  * Command

		`"$PWM?*\n"`

  * Example response:

		`'[-70, 90]\n'`

* Get IR values:
  * Command

		`"$IRVAL?*\n"`

  * Example response:

		`'[402.0, 54.0, 33.0, 805.0, 24.0]\n'`

* Get raw encoder values (value stored in encoder, only resets on power cycle):
  * Command

		`"$ENRAW?*\n"`

  * Example response:

		`'[1001, -5362]\n'`

* Get current encoder values:
  * Command

        `"$ENVAL?*\n"`

  * Example response:

        `'[-16, 12]\n'`

* Set encoder values
  * Command

      `$ENVAL=10,-10*\n"`

* Get encoder value offsets from raw values:
  * Command

      `"$ENOFFSET?*\n"`

  * Example response:

      `'[10, 20]\n'`

* Set encoder value offsets from raw values:
  * Command

      `"$ENOFFSET=0,0*\n"`

* Reset encoder values to zero:
  * Command

      `"$ENRESET*\n"`

* Get current encoder velocity values:
  * Command

        `"$ENVEL?*\n"`

  * Example response:

        `'[-16, 12]\n'`

* End program QuickBot program
  * Command:

 		`"$END*\n"`

