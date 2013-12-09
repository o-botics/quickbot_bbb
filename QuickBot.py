#!/usr/bin/python

import sys
import time
import math
import re
import numpy as np

# import estimation.kalman as kalman

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import serial
import socket

# Constants
LEFT = 0
RIGHT = 1
MIN = 0
MAX = 1

def convertHEXtoDEC(hexString, N):
    # Return 2's compliment of hexString
    for hexChar in hexString:
        asciiNum = ord(hexChar)
        if not ((asciiNum >= 48 and asciiNum <= 57) or \
             (asciiNum >= 65 and asciiNum <= 70) or \
             (asciiNum >= 97 and asciiNum <= 102)):
             val = float('nan')
             return val

    if len(hexString) == N:
        val = int(hexString, 16)
        bits = 4*len(hexString)
        if  (val & (1<<(bits-1))) != 0:
            val = val - (1<<bits)
        return val

class QuickBot:
    """The QuickBot Class"""

    # === Class Properties ===
    # Parameters
    sampleTime = 100.0 / 1000.0

    # Pins
    ledPin = 'USR1'

    # Motor Pins -- (LEFT, RIGHT)
    dir1Pin = ('P8_10', 'P8_14')
    dir2Pin = ('P8_12', 'P8_16')
    pwmPin = ('P9_14', 'P9_16')

    # State -- (LEFT, RIGHT)
    pwm = [0, 0]
    encoderVal = [float('nan'), float('nan')]
    encoderVel = [float('nan'), float('nan')]

    # Constraints
    pwmLimits = [-100, 100] # [min, max]

    # Variables
    runFlag = True
    ledFlag = True
    cmdBuffer = ''
    encoderBuffer = ['', '']

    # UDP

    baseIP = '192.168.7.1'
    robotIP = '192.168.7.2'

    port = 5005
    robotSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robotSocket.setblocking(False)

    # Serial
    # TTYO1: Rx=P9_26  Tx=P9_24
    # TTYO2: Rx=P9_22  Tx=P9_21
    # TTYO4: Rx=P9_11  Tx=P9_13
    # TTYO5: Rx=P9_38  Tx=P9_38
    encoderSerial = (serial.Serial(port = '/dev/ttyO4', baudrate = 38400, timeout = .1), \
                     serial.Serial(port = '/dev/ttyO2', baudrate = 38400, timeout = .1))

    # === Class Methods ===
    # Constructor
    def __init__(self, baseIP, robotIP):

        # Initialize GPIO pins
        GPIO.setup(self.dir1Pin[LEFT], GPIO.OUT)
        GPIO.setup(self.dir2Pin[LEFT], GPIO.OUT)
        GPIO.setup(self.dir1Pin[RIGHT], GPIO.OUT)
        GPIO.setup(self.dir2Pin[RIGHT], GPIO.OUT)

        GPIO.setup(self.ledPin, GPIO.OUT)

        # Initialize PWM pins: PWM.start(channel, duty, freq=2000, polarity=0)
        PWM.start(self.pwmPin[LEFT], 0)
        PWM.start(self.pwmPin[RIGHT], 0)

        # Set motor speed to 0
        self.setPWM([0, 0])

        # Set IP addresses
        self.baseIP = baseIP
        self.robotIP = robotIP
        self.robotSocket.bind((self.robotIP, self.port))

    # Getters and Setters
    def setPWM(self, pwm):
        # [leftSpeed, rightSpeed]: 0 is off, caps at min and max values

        self.pwm[LEFT] = min(max(pwm[LEFT], self.pwmLimits[MIN]), self.pwmLimits[MAX])
        self.pwm[RIGHT] = min(max(pwm[RIGHT], self.pwmLimits[MIN]), self.pwmLimits[MAX])
        print 'Setting motor PWMs to: left = ' + str(self.pwm[LEFT]) + ' and right = ' + str(self.pwm[RIGHT])

        # Left motor
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

        # Right motor
        if self.pwm[RIGHT] > 0:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.HIGH)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], abs(self.pwm[RIGHT]))
        elif self.pwm[RIGHT] < 0:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.LOW)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.HIGH)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], abs(self.pwm[RIGHT]))
        else:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.LOW)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], 0)

    # Methods
    def run(self):
        # Turn off velocity output


        # Kp = 0
        # Ki = 0.05

        # k = 100
        # i = -1
        # t = 0.
        # dt = 0.
        # dtBar = 0.1

        # xBar = dtBar
        # u = 0.
        # x = 0.
        # P = 1.
        # y = xBar + u

        # Phi = 1.
        # H = 1.
        # W = .00000001
        # V = .00021035
        # E = [0] * k

        while self.runFlag == True:
            tStart = time.time()

            self.update()
            if self.ledFlag == True:
                self.ledFlag = False
                GPIO.output(self.ledPin, GPIO.HIGH)
            else:
                self.ledFlag = True
                GPIO.output(self.ledPin, GPIO.LOW)

            # # Stabilize sample time
            # i = i + 1
            # t = t + dt

            # if i >= 1:
            #     z = dt
            #     (x, P, K, w, z, chi2) = kalman.filter(x, P, Phi, H, W, V, z)
            #     e = xBar - x
            #     if i < k-1:
            #         E[i] = e
            #         u = Kp*e + Ki*sum(E[0:i+1])
            #     else:
            #         E[i%k] = e
            #         u = Kp*e + Ki*sum(E)

            #     y = xBar + u

            # time.sleep(y)
            # dt = time.time() - tStart
            # print dt
            # print x

    def cleanup(self):
        self.setPWM([0, 0])
        self.robotSocket.close()
        self.encoderSerial[LEFT].close()
        self.encoderSerial[RIGHT].close()
        GPIO.cleanup()
        PWM.cleanup()

    def update(self):
        encoderUpdateFlag = False

        encoderUpdateFlag = self.parseEncoderBuffer(LEFT)

        encoderUpdateFlag = encoderUpdateFlag or self.parseEncoderBuffer(RIGHT)

        self.parseCmdBuffer()

        if (encoderUpdateFlag):
            print 'Pos: [' + ', '.join(map(str, self.encoderVal)) + ']' + \
              '\tVel: [' + ', '.join(map(str, self.encoderVel)) + ']' + \
              '\tBuf: [' + str(self.encoderSerial[LEFT].inWaiting()) + ', ' + \
              str(self.encoderSerial[RIGHT].inWaiting()) + ']'

    def parseCmdBuffer(self):
        try:
            line = self.robotSocket.recv(1024)
        except socket.error as msg:
            line = ''
        self.cmdBuffer += line

        bufferPattern = r'\$[^\$\*]*?\*' # String contained within $ and * symbols with no $ or * symbols in it
        bufferRegex = re.compile(bufferPattern)
        bufferResult = bufferRegex.search(self.cmdBuffer)

        if bufferResult:
            msg = bufferResult.group()
            print msg
            self.cmdBuffer = ''

            msgPattern = r'\$(?P<CMD>[A-Z]{3,})(?P<SET>=?)(?P<QUERY>\??)(?(2)(?P<ARGS>.*)).*\*'
            msgRegex = re.compile(msgPattern)
            msgResult = msgRegex.search(msg)

            if msgResult.group('CMD') == 'CHECK':
                self.robotSocket.sendto('Hello from QuickBot\n',(self.baseIP, self.port))

            elif msgResult.group('CMD') == 'PWM':
                if msgResult.group('QUERY'):
                    self.robotSocket.sendto(str(self.pwm) + '\n',(self.baseIP, self.port))

                elif msgResult.group('SET') and msgResult.group('ARGS'):
                    args = msgResult.group('ARGS')
                    pwmArgPattern = r'(?P<LEFT>[-]?\d+),(?P<RIGHT>[-]?\d+)'
                    pwmRegex = re.compile(pwmArgPattern)
                    pwmResult = pwmRegex.match(args)
                    if pwmResult:
                        pwm = [int(pwmRegex.match(args).group('LEFT')), \
                        int(pwmRegex.match(args).group('RIGHT'))]
                        self.setPWM(pwm)

            elif msgResult.group('CMD') == 'ENVAL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.encoderVal)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))

            elif msgResult.group('CMD') == 'ENVEL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.encoderVel)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))

            elif msgResult.group('CMD') == 'UPDATE':
                if msgResult.group('SET') and msgResult.group('ARGS'):
                    args = msgResult.group('ARGS')
                    pwmArgPattern = r'(?P<LEFT>[-]?\d+),(?P<RIGHT>[-]?\d+)'
                    pwmRegex = re.compile(pwmArgPattern)
                    pwmResult = pwmRegex.match(args)
                    if pwmResult:
                        pwm = [int(pwmRegex.match(args).group('LEFT')), \
                        int(pwmRegex.match(args).group('RIGHT'))]
                        self.setPWM(pwm)

                    reply = '[' + ', '.join(map(str, self.encoderVal)) + ', ' \
                      + ', '.join(map(str, self.encoderVel)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))

            elif msgResult.group('CMD') == 'END':
                print 'Quitting QuickBot run loop'
                self.runFlag = False

            elif msgResult.group('CMD') == 'DUMP':
                self.dumpEncoderBuffer(LEFT)

    def dumpEncoderBuffer(self, side):
        while self.encoderSerial[side].inWaiting() > 0:
            print self.encoderSerial[side].readline()
            print self.encoderSerial[side].inWaiting()

    def parseEncoderBuffer(self, side):
        encoderUpdateFlag = False

        bytesInWaiting = self.encoderSerial[side].inWaiting()
        if (bytesInWaiting > 0):
            self.encoderBuffer[side] += self.encoderSerial[side].read(bytesInWaiting)

            if len(self.encoderBuffer[side]) > 30:
                self.encoderBuffer[side] = self.encoderBuffer[side][-30:]

            if len(self.encoderBuffer[side]) >= 15:
                DPattern = r'D([0-9A-F]{8})'
                DRegex = re.compile(DPattern)
                DResult = DRegex.findall(self.encoderBuffer[side])
                if len(DResult) >= 1:
                    val = convertHEXtoDEC(DResult[-1], 8)
                    if not math.isnan(val):
                        self.encoderVal[side] = val
                        encoderUpdateFlag = True

                VPattern = r'V([0-9A-F]{4})'
                VRegex = re.compile(VPattern)
                VResult = VRegex.findall(self.encoderBuffer[side])
                if len(VResult) >= 1:
                    vel = convertHEXtoDEC(VResult[-1], 4)
                    if not math.isnan(vel):
                        self.encoderVel[side] = vel
                        encoderUpdateFlag = True

            return encoderUpdateFlag
