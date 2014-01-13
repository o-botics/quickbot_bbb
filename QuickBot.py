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
import socket
import threading

# import numpy as np
# import estimation.kalman as kalman

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC


# Constants
LEFT = 0
RIGHT = 1
MIN = 0
MAX = 1

ENC_LEFT_VAL = 0
ENC_LEFT_VAL_LOCK = threading.Lock()
ENC_RIGHT_VAL = 0
ENC_RIGHT_VAL_LOCK = threading.Lock()
ADC_LOCK = threading.Lock()
RUN_FLAG = True
RUN_FLAG_LOCK = threading.Lock()

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

class QuickBot():
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
    encoderPin = ('P9_39','P9_37')

    # State -- (LEFT, RIGHT)
    pwm = [0, 0]
    irVal = [0, 0, 0, 0, 0]
    encoderVal = [0, 0]
    encoderVel = [0.0, 0.0]

    # Constraints
    pwmLimits = [-100, 100] # [min, max]

    # Variables
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
        
        # Initialize encoders
        self.encoder = []
        self.encoder.append(Encoder(LEFT,self.encoderPin[LEFT]))
        self.encoder.append(Encoder(RIGHT,self.encoderPin[RIGHT]))

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
        global RUN_FLAG
        self.encoder[LEFT].start()
        #self.encoder[RIGHT].start()
        
        while RUN_FLAG == True:
            self.update()
            # Flash BBB LED
            if self.ledFlag == True:
                self.ledFlag = False
                GPIO.output(self.ledPin, GPIO.HIGH)
            else:
                self.ledFlag = True
                GPIO.output(self.ledPin, GPIO.LOW)
        self.cleanup()        
        return

    def cleanup(self):
        print "Clean up"
        self.setPWM([0, 0])
        self.robotSocket.close()
        GPIO.cleanup()
        PWM.cleanup()

    def update(self):
        #self.readIRValues()
        self.readEncoderValues()
        self.parseCmdBuffer()

    def parseCmdBuffer(self):
        global RUN_FLAG
        try:
            line = self.robotSocket.recv(1024)
        except socket.error as msg:
            #print msg
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
                RUN_FLAG_LOCK.acquire()
                RUN_FLAG = False
                RUN_FLAG_LOCK.release()

    def readIRValues(self):
        for i in range(0,len(self.irPin)):
            # ADC_LOCK.acquire()
            # self.irVal[i] = ADC.read_raw(self.irPin[i])
            time.sleep(1.0/1000.0)
            # ADC_LOCK.release()
            # print "IR " + str(i) + ": " + str(self.irVal[i])
            
    def readEncoderValues(self):
        self.encoderVal[LEFT] = ENC_LEFT_VAL
        self.encoderVal[RIGHT] = ENC_RIGHT_VAL
        print "ENC_LEFT_VAL: " + str(ENC_LEFT_VAL) + "  ENC_RIGHT_VAL: " + str(ENC_RIGHT_VAL)
            

class Encoder(threading.Thread):
    """The Encoder Class"""
    
    # === Class Properties ===
    # Parameters
    sampleTime = 0.001
    threshold = 1450
    tickPerRev = 16
    
    # Encoder side
    # side
    
    # ADC Pins
    # pin
    
    # State
    t0 = -1
    t = -1
    tPrev = -1
    val = -1
    cog = -1
    tick = 0
    dir = 1
    pos = 0
    vel = 0
    
    # === Class Methods ===
    # Constructor
    def __init__(self,side=LEFT,pin='P9_39'):
        
        # Initialize thread
        threading.Thread.__init__(self)
        
        # Initialize ADC
        ADC.setup()
        
        # Set properties
        self.pin = pin
        self.side = side

    # Methods
    def run(self):
        global RUN_FLAG
        global ENC_LEFT_VAL
        global ENC_RIGHT_VAL
        cnt = 0
        self.t0 = time.time()
        
        while RUN_FLAG:
            try:
                self.sample()
            except:
                RUN_FLAG_LOCK.acquire()
                RUN_FLAG = False
                RUN_FLAG_LOCK.release()
                
            if self.tick == 1:
                if self.side == LEFT:
                    print "Left hello"
                    ENC_LEFT_VAL_LOCK.acquire()
                    ENC_LEFT_VAL = ENC_LEFT_VAL + 1
                    ENC_LEFT_VAL_LOCK.release()
                else:
                    print "Right hello"
                    ENC_RIGHT_VAL_LOCK.acquire()
                    ENC_RIGHT_VAL = ENC_RIGHT_VAL + 1
                    ENC_RIGHT_VAL_LOCK.release()
#                 print 'Time: ' + str(self.t) + \
#                   '\tPos: ' + str(self.pos) + \
#                   '\tVel: ' + str(self.vel)
                
            time.sleep(self.sampleTime)
            
        return
    
    def sample(self):
        t = time.time() - self.t0
        ADC_LOCK.acquire()
        self.val = ADC.read_raw(self.pin)
        ADC_LOCK.release()
                
        cogPrev = self.cog
        if self.val >= self.threshold:
            self.cog = 1
        else:
            self.cog = 0
         
        if cogPrev == 0 and self.cog == 1:
            # Tick
            self.tick = 1
            self.pos = self.pos + self.dir
            if self.tPrev != -1:
                self.vel = self.dir * (t - self.t)**(-1.0) / (self.tickPerRev)
            self.tPrev = self.t
            self.t = t
        else:
            # No Tick
            self.tick = 0 
