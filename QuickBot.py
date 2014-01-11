#!/usr/bin/python
"""
@brief QuickBot class for Beaglebone Black

@author Rowland O'Flaherty 
@date 08/27/2013
"""

import sys
import time
import math
import re
# import numpy as np

# import estimation.kalman as kalman

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
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
    sampleTime = 50.0 / 1000.0

    # Pins
    ledPin = 'USR1'

    # Motor Pins -- (LEFT, RIGHT)
    dir1Pin = ('P8_12', 'P8_14')
    dir2Pin = ('P8_10', 'P8_16')
    pwmPin = ('P9_14', 'P9_16')

    # ADC Pins
    irPin = ('P9_35', 'P9_33', 'P9_40', 'P9_36', 'P9_38')
    # irPin = ('P9_35', 'P9_33', 'P9_40', 'P9_36')
    encoderPin = ('P9_37','P9_39')

    # State -- (LEFT, RIGHT)
    pwm = [0, 0]
    irVal = [0, 0, 0, 0, 0]
    encoderVal = [float('nan'), float('nan')]
    encoderVel = [float('nan'), float('nan')]

    # Constraints
    pwmLimits = [-100, 100] # [min, max]

    # Variables
    runFlag = True
    ledFlag = True
    cmdBuffer = ''

    # UDP
    baseIP = '192.168.7.1'
    robotIP = '192.168.7.2'

    port = 5005
    robotSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robotSocket.setblocking(False)

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

        # Initialize ADC
        ADC.setup()

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

    # Methods
    def run(self):
        cnt = 0
        while self.runFlag == True:
            self.update()
            # Flash BBB LED
            if self.ledFlag == True:
                self.ledFlag = False
                GPIO.output(self.ledPin, GPIO.HIGH)
            else:
                self.ledFlag = True
                GPIO.output(self.ledPin, GPIO.LOW)

    def cleanup(self):
        self.setPWM([0, 0])
        self.robotSocket.close()
        GPIO.cleanup()
        PWM.cleanup()

    def update(self):
        self.readIRValues()
        self.readEncoderValues()

        self.parseCmdBuffer()

    def parseCmdBuffer(self):
        try:
            line = self.robotSocket.recv(1024)
        except socket.error as msg:
            return
        
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

            elif msgResult.group('CMD') == 'IRVAL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.irVal)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))

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

    def readIRValues(self):
        for i in range(0,len(self.irPin)):            
            self.irVal[i] = ADC.read_raw(self.irPin[i])
            time.sleep(1.0/1000.0)
            # print "IR " + str(i) + ": " + str(self.irVal[i])
            
    def readEncoderValues(self):
        for i in range(0,len(self.encoderPin)):
            self.encoderVal[i] = ADC.read_raw(self.encoderPin[i])
            time.sleep(1.0/1000.0)
            # print "ENC " + str(i) + ": " + str(self.irVal[i])
        
