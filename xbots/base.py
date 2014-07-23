"""
@brief Base robot class

@author Josip Delic (delijati.net)
@author Rowland O'Flaherty (rowlandoflaherty.com)
@date 04/23/2014
@version: 1.0
@copyright: Copyright (C) 2014, see the LICENSE file
"""

import sys
import time
import re
import socket
import threading

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM

LEFT = 0
RIGHT = 1
MIN = 0
MAX = 1

DEBUG = False
VERBOSE = True

class BaseBot(object):
    """
    Base robot class. Mainly handles initialization and messages passing.
    """

    # Parameters
    sample_time = 20.0 / 1000.0
    pwm_freq = 2000

    # Variables
    led_flag = True
    cmdBuffer = ''

    # Motor Pins -- (LEFT, RIGHT)
    dir1Pin = ("", "")
    dir2Pin = ("", "")
    pwmPin = ("", "")

    # Led pin
    led = ""

    # State PWM -- (LEFT, RIGHT)
    pwm = [0, 0]

    # Constraints
    pwmLimits = [-100, 100]  # [min, max]

    # UDP
    port = 5005
    robotSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robotSocket.setblocking(False)

    def __init__(self, base_ip, robot_ip):
        # Run flag
        self.run_flag = True

        # Initialize GPIO pins
        self._setup_gpio()

        # Initialize PWM pins: PWM.start(channel, duty, freq=2000, polarity=0)
        self._init_pwm()

        # Set motor speed to 0
        self.set_pwm([0, 0])

        # Set IP addresses
        self.base_ip = base_ip
        self.robot_ip = robot_ip
        self.robotSocket.bind((self.robot_ip, self.port))

        # Initialize command parsing thread
        self.cmd_parsing_thread = threading.Thread(target=parse_cmd,
                                                   args=(self,))
        self.cmd_parsing_thread.daemon = True

    def _setup_gpio(self):
        """Initialize GPIO pins"""
        GPIO.setup(self.dir1Pin[LEFT], GPIO.OUT)
        GPIO.setup(self.dir2Pin[LEFT], GPIO.OUT)
        GPIO.setup(self.dir1Pin[RIGHT], GPIO.OUT)
        GPIO.setup(self.dir2Pin[RIGHT], GPIO.OUT)
        GPIO.setup(self.led, GPIO.OUT)

    def _init_pwm(self):
        """
        Initialize PWM pins
        """

        # It is currently not possible to set frequency for two PWM
        # a maybe solution patch pwm_test.c
        # https://github.com/SaadAhmad/beaglebone-black-cpp-PWM
        PWM.start(self.pwmPin[LEFT], 0)
        PWM.start(self.pwmPin[RIGHT], 0)

    def set_pwm(self, pwm):
        """ Set motor PWM values """
        # [leftSpeed, rightSpeed]: 0 is off, caps at min and max values

        self.set_pwm_left(pwm[LEFT])
        self.set_pwm_right(pwm[RIGHT])

    def set_pwm_left(self, pwm_left):
        """ Set left motor PWM value """

        self.pwm[LEFT] = min(
            max(pwm_left, self.pwmLimits[MIN]), self.pwmLimits[MAX])

        if self.pwm[LEFT] > 0:
            GPIO.output(self.dir1Pin[LEFT], GPIO.LOW)
            GPIO.output(self.dir2Pin[LEFT], GPIO.HIGH)
            PWM.set_duty_cycle(self.pwmPin[LEFT], abs(self.pwm[LEFT]))
        elif self.pwm[LEFT] < 0:
            GPIO.output(self.dir1Pin[LEFT], GPIO.HIGH)
            GPIO.output(self.dir2Pin[LEFT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[LEFT], abs(self.pwm[LEFT]))
        else:
            GPIO.output(self.dir1Pin[LEFT], GPIO.LOW)
            GPIO.output(self.dir2Pin[LEFT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[LEFT], 0)

    def set_pwm_right(self, pwm_right):
        """ Set right motor PWM value """

        self.pwm[RIGHT] = min(
            max(pwm_right, self.pwmLimits[MIN]), self.pwmLimits[MAX])

        if self.pwm[RIGHT] > 0:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.LOW)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.HIGH)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], abs(self.pwm[RIGHT]))
        elif self.pwm[RIGHT] < 0:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.HIGH)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], abs(self.pwm[RIGHT]))
        else:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.LOW)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], 0)

    def get_pwm(self):
        """ Get motor PWM values """
        return self.pwm

    def start_threads(self):
        """ Start all threads """
        self.cmd_parsing_thread.start()

    def update(self):
        """ Update which occures once per cycle of the run loop """
        # Flash BBB LED
        if self.led_flag is True:
            self.led_flag = False
            GPIO.output(self.led, GPIO.HIGH)
        else:
            self.led_flag = True
            GPIO.output(self.led, GPIO.LOW)

    def run(self):
        """ The run loop """
        # Start threads
        self.start_threads()

        # Run loop
        while self.run_flag:
            self.update()
            time.sleep(self.sample_time)
        self.cleanup()
        return

    def end_run(self):
        """ End the run loop. Gives time for threads to receive run_flag. """
        self.run_flag = False
        time.sleep(2*self.sample_time)


    def cleanup(self):
        """ Clean up before shutting down. """
        sys.stdout.write("Shutting down...")
        self.set_pwm([0, 0])
        self.robotSocket.close()
        GPIO.cleanup()
        PWM.cleanup()
        if DEBUG:
            pass
            # tictocPrint()
            # self.writeBuffersToFile()
        sys.stdout.write("Done\n")

    # def writeBuffersToFile(self):
    #     matrix = map(list, zip(*[self.encTimeRec[LEFT], self.encValRec[LEFT],
                             # self.encPWMRec[LEFT], self.encNNewRec[LEFT],
                             # self.encTickStateRec[LEFT],
                             # self.enc_posRec[LEFT],
                             # self.encVelRec[LEFT],
                             # self.encThresholdRec[LEFT],
                             # self.encTimeRec[RIGHT],
                             # self.encValRec[RIGHT],
                             # self.encPWMRec[RIGHT], self.encNNewRec[RIGHT],
                             # self.encTickStateRec[RIGHT],
                             # self.enc_posRec[RIGHT], self.encVelRec[RIGHT],
                             # self.encThresholdRec[RIGHT]]))
    #     s = [[str(e) for e in row] for row in matrix]
    #     lens = [len(max(col, key=len)) for col in zip(*s)]
    #     fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
    #     table = [fmt.format(*row) for row in s]
    #     f = open('output.txt', 'w')
    #     f.write('\n'.join(table))
    #     f.close()
    #     print "Wrote buffer to output.txt"


def parse_cmd(self):
    """ Command parser """
    try:
        while self.run_flag:
            try:
                line = self.robotSocket.recv(1024)
            except socket.error as msg:
                continue

            self.cmdBuffer += line

            # String contained within $ and * (with no $ or * symbols in it)
            buf_pattern = r'\$[^\$\*]*?\*'
            buf_regex = re.compile(buf_pattern)
            buf_result = buf_regex.search(self.cmdBuffer)

            if buf_result:
                msg = buf_result.group()
                print msg
                self.cmdBuffer = ''

                cmd_pattern = r'(?P<CMD>[A-Z]{3,})'
                set_pattern = r'(?P<SET>=?)'
                query_pattern = r'(?P<QUERY>\??)'
                arg_pattern = r'(?(2)(?P<ARGS>.*))'
                msg_pattern = r'\$' + \
                    cmd_pattern + \
                    set_pattern + \
                    query_pattern + \
                    arg_pattern + \
                    r'.*\*'

                msg_regex = re.compile(msg_pattern)
                msg_result = msg_regex.search(msg)

                if msg_result.group('CMD') == 'CHECK':
                    self.robotSocket.sendto(
                        'Hello from QuickBot\n', (self.base_ip, self.port))

                elif msg_result.group('CMD') == 'PWM':
                    if msg_result.group('QUERY'):
                        if VERBOSE:
                            print str(self.get_pwm())
                        self.robotSocket.sendto(str(self.get_pwm()) + '\n',
                                                (self.base_ip, self.port))

                    elif msg_result.group('SET') and msg_result.group('ARGS'):
                        args = msg_result.group('ARGS')
                        pwm_pattern = r'(?P<LEFT>[-]?\d+),(?P<RIGHT>[-]?\d+)'
                        pwm_regex = re.compile(pwm_pattern)
                        pwm_result = pwm_regex.match(args)
                        if pwm_result:
                            pwm = [int(pwm_result.group('LEFT')), \
                                    int(pwm_result.group('RIGHT'))]
                            self.set_pwm(pwm)

                    self.robotSocket.sendto(str(self.get_pwm()) + '\n',
                                            (self.base_ip, self.port))

                elif msg_result.group('CMD') == 'IRVAL':
                    if msg_result.group('QUERY'):
                        reply = '[' + ', '.join(map(str, self.get_ir())) + ']'
                        print 'Sending: ' + reply
                        self.robotSocket.sendto(
                            reply + '\n', (self.base_ip, self.port))

                elif msg_result.group('CMD') == 'ULTRAVAL':
                    if msg_result.group('QUERY'):
                        reply = '[' + ', '.join(map(str, self.ultraVal)) + ']'
                        print 'Sending: ' + reply
                        self.robotSocket.sendto(
                            reply + '\n', (self.base_ip, self.port))

                elif msg_result.group('CMD') == 'WHEELANG':
                    if msg_result.group('QUERY'):
                        print 'Sending: ' + str(self.get_wheel_ang())
                        self.robotSocket.sendto(
                            str(self.get_wheel_ang()) +
                            '\n', (self.base_ip, self.port))

                    elif msg_result.group('SET') and msg_result.group('ARGS'):
                        args = msg_result.group('ARGS')
                        arg_pattern = \
                        r'(?P<LEFT>[-]?\d+[\.]?\d*),(?P<RIGHT>[-]?\d+[\.]?\d*)'
                        regex = re.compile(arg_pattern)
                        result = regex.match(args)
                        if result:
                            pos = [float(regex.match(args).group('LEFT')), \
                                float(regex.match(args).group('RIGHT'))]
                            self.set_wheel_ang(pos)

                elif msg_result.group('CMD') == 'ENVAL':
                    if msg_result.group('QUERY'):
                        reply = \
                            '[' + ', '.join(map(str, self.get_enc_val())) + ']'
                        print 'Sending: ' + reply
                        self.robotSocket.sendto(
                            reply + '\n', (self.base_ip, self.port))

                    elif msg_result.group('SET') and msg_result.group('ARGS'):
                        args = msg_result.group('ARGS')
                        arg_pattern = \
                        r'(?P<LEFT>[-]?\d+[\.]?\d*),(?P<RIGHT>[-]?\d+[\.]?\d*)'
                        regex = re.compile(arg_pattern)
                        result = regex.match(args)
                        if result:
                            enc_pos = [float(regex.match(args).group('LEFT')), \
                                     float(regex.match(args).group('RIGHT'))]
                            self.set_enc_val(enc_pos)

                elif msg_result.group('CMD') == 'ENRAW':
                    if msg_result.group('QUERY'):
                        reply = \
                            '[' + ', '.join(map(str, self.get_enc_raw())) + ']'
                        print 'Sending: ' + reply
                        self.robotSocket.sendto(
                            reply + '\n', (self.base_ip, self.port))

                elif msg_result.group('CMD') == 'ENOFFSET':
                    if msg_result.group('QUERY'):
                        reply = '[' + \
                            ', '.join(map(str, self.get_enc_offset())) + ']'
                        print 'Sending: ' + reply
                        self.robotSocket.sendto(
                            reply + '\n', (self.base_ip, self.port))

                    elif msg_result.group('SET') and msg_result.group('ARGS'):
                        args = msg_result.group('ARGS')
                        arg_pattern = \
                        r'(?P<LEFT>[-]?\d+[\.]?\d*),(?P<RIGHT>[-]?\d+[\.]?\d*)'
                        regex = re.compile(arg_pattern)
                        result = regex.match(args)
                        if result:
                            offset = [float(regex.match(args).group('LEFT')), \
                                     float(regex.match(args).group('RIGHT'))]
                            self.set_enc_offset(offset)

                elif msg_result.group('CMD') == 'ENVEL':
                    if msg_result.group('QUERY'):
                        reply = \
                            '[' + ', '.join(map(str, self.get_enc_vel())) + ']'
                        print 'Sending: ' + reply
                        self.robotSocket.sendto(
                            reply + '\n', (self.base_ip, self.port))

                    elif msg_result.group('SET') and msg_result.group('ARGS'):
                        args = msg_result.group('ARGS')
                        arg_pattern = \
                        r'(?P<LEFT>[-]?\d+[\.]?\d*),(?P<RIGHT>[-]?\d+[\.]?\d*)'
                        regex = re.compile(arg_pattern)
                        result = regex.match(args)
                        if result:
                            enc_vel = [float(regex.match(args).group('LEFT')), \
                            float(regex.match(args).group('RIGHT'))]
                            self.set_enc_vel(enc_vel)

                elif msg_result.group('CMD') == 'WHEELANGVEL':
                    if msg_result.group('QUERY'):
                        reply = \
                            '[' + ', '.join(map(str, self.get_wheel_ang_vel())) + ']'
                        print 'Sending: ' + reply
                        self.robotSocket.sendto(
                            reply + '\n', (self.base_ip, self.port))

                    elif msg_result.group('SET') and msg_result.group('ARGS'):
                        args = msg_result.group('ARGS')
                        arg_pattern = \
                        r'(?P<LEFT>[-]?\d+[\.]?\d*),(?P<RIGHT>[-]?\d+[\.]?\d*)'
                        regex = re.compile(arg_pattern)
                        result = regex.match(args)
                        if result:
                            wheel_ang_vel = [float(regex.match(args).group('LEFT')), \
                                     float(regex.match(args).group('RIGHT'))]
                            self.set_wheel_ang_vel(wheel_ang_vel)

                elif msg_result.group('CMD') == 'ENRESET':
                    self.reset_enc_val()
                    reply = \
                            '[' + ', '.join(map(str, self.get_enc_val())) + ']'
                    print 'Encoder values reset to ' + reply

                elif msg_result.group('CMD') == 'UPDATE':
                    if msg_result.group('SET') and msg_result.group('ARGS'):
                        args = msg_result.group('ARGS')
                        pwm_pattern = r'(?P<LEFT>[-]?\d+),(?P<RIGHT>[-]?\d+)'
                        pwm_regex = re.compile(pwm_pattern)
                        pwm_result = pwm_regex.match(args)
                        if pwm_result:
                            pwm = [int(pwm_regex.match(args).group('LEFT')), \
                                    int(pwm_regex.match(args).group('RIGHT'))]
                            self.set_pwm(pwm)

                        reply = '[' + ', '.join(map(str, self.enc_pos)) + ', ' \
                            + ', '.join(map(str, self.encVel)) + ']'
                        print 'Sending: ' + reply
                        self.robotSocket.sendto(
                            reply + '\n', (self.base_ip, self.port))

                elif msg_result.group('CMD') == 'END':
                    self.end_run()

                else:
                    print 'Invalid: ' + msg
    except:
        self.end_run()
        raise
