#!/usr/bin/python
"""
@brief QuickBot class for Beaglebone Black

@author Rowland O'Flaherty 
@date 08/27/2013
"""
import os
import sys
import time
import math
import re
import socket
import threading

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC


# Constants
LEFT = 0
RIGHT = 1
MIN = 0
MAX = 1

ADCTIME = 0.001

ENC_BUF_SIZE = 2**13

ENC_IND = [0, 0]
ENC_TIME = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]
ENC_VAL = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]

ADC_LOCK = threading.Lock()

RUN_FLAG = True
RUN_FLAG_LOCK = threading.Lock()

class QuickBot():
    """The QuickBot Class"""

    # === Class Properties ===
    # Parameters
    sampleTime = 25.0 / 1000.0

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
    ithIR = 0
    
    encoderVal = [0, 0]
    encoderVel = [0.0, 0.0]
    encTimeBuf = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]
    encValBuf = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]
    encPWMBuf = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]
    encNNewBuf = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]
    encBufInd0 = [0, 0]
    encBufInd1 = [0, 0]

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
        self.encoderRead = encoderRead(self.encoderPin)

        # Set IP addresses
        self.baseIP = baseIP
        self.robotIP = robotIP
        self.robotSocket.bind((self.robotIP, self.port))

    # Getters and Setters
    def setPWM(self, pwm):
        global ENC_DIR
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
        self.encoderRead.start()
        
        while RUN_FLAG == True:
            tStart = time.time()

            self.update()
             
            # Flash BBB LED
            if self.ledFlag == True:
                self.ledFlag = False
                GPIO.output(self.ledPin, GPIO.HIGH)
            else:
                self.ledFlag = True
                GPIO.output(self.ledPin, GPIO.LOW)
            time.sleep(self.sampleTime)
            
            tEnd = time.time()            
#             print("Time: ") + str(tEnd - tStart)         
            
        self.cleanup()        
        return

    def cleanup(self):
        print "Clean up"
        self.setPWM([0, 0])
        self.robotSocket.close()
        GPIO.cleanup()
        PWM.cleanup()
        self.writeBufferToFile()

    def update(self):
        self.readIRValues()
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
        prevVal = self.irVal[self.ithIR]
        ADC_LOCK.acquire()
        self.irVal[self.ithIR] = ADC.read_raw(self.irPin[self.ithIR])
        time.sleep(ADCTIME)
        ADC_LOCK.release()
        
        if self.irVal[self.ithIR] >= 1000:
                self.irVal[self.ithIR] = prevVal
        
#         if self.ithIR == 4:
#             print "IR " + str(self.ithIR) + ": " + str(self.irVal[self.ithIR])
        
        self.ithIR = ((self.ithIR+1) % 5)
        
            
    def readEncoderValues(self):
        # Fill buffers
        for side in range(0,2):
            self.encBufInd0[side] = self.encBufInd1[side]
            self.encBufInd1[side] = ENC_IND[side]
            ind0 = self.encBufInd0[side]
            ind1 = self.encBufInd1[side]
            if ind0 < ind1:    
                self.encTimeBuf[side][ind0:ind1] = ENC_TIME[side][ind0:ind1]
                self.encValBuf[side][ind0:ind1] = ENC_VAL[side][ind0:ind1]
                self.encPWMBuf[side][ind0:ind1] = [self.pwm[side]]*(ind1-ind0)
                self.encNNewBuf[side][ind0:ind1] = [(ind1-ind0)]*(ind1-ind0)
            elif ind0 > ind1:
                self.encTimeBuf[side][ind0:ENC_BUF_SIZE] = ENC_TIME[side][ind0:ENC_BUF_SIZE]
                self.encValBuf[side][ind0:ENC_BUF_SIZE] = ENC_VAL[side][ind0:ENC_BUF_SIZE]
                self.encPWMBuf[side][ind0:ENC_BUF_SIZE] = [self.pwm[side]]*(ENC_BUF_SIZE-ind0)
                self.encNNewBuf[side][ind0:ENC_BUF_SIZE] = [(ENC_BUF_SIZE-ind0+ind0)]*(ENC_BUF_SIZE-ind0)
                if ind1 > 0:
                    self.encTimeBuf[side][0:ind1] = ENC_VAL[side][0:ind1]
                    self.encValBuf[side][0:ind1] = ENC_VAL[side][0:ind1]
                    self.encPWMBuf[side][0:ind1] = [self.pwm[side]]*ind1
                    self.encNNewBuf[side][0:ind1] = [(ENC_BUF_SIZE-ind0+ind1)]*ind1
            
#         print "LEFT: " + str(self.encBufInd1[LEFT] - self.encBufInd0[LEFT]) + \
#             " RIGHT: " + str(self.encBufInd1[RIGHT] - self.encBufInd0[RIGHT])
#         self.encoderVal[LEFT] = ENC_VAL_LEFT[ENC_IND_LEFT]
#         self.encoderVal[RIGHT] = ENC_VAL_RIGHT[ENC_IND_RIGHT]
        
#         print "ENC_LEFT_VAL: " + str(self.encoderVal[LEFT]) + " ENC_RIGHT_VAL: " + str(self.encoderVal[RIGHT])
        
    def writeBufferToFile(self):
        matrix = map(list, zip(*[self.encTimeBuf[LEFT], self.encValBuf[LEFT], self.encPWMBuf[LEFT], self.encNNewBuf[LEFT], \
                                 self.encTimeBuf[RIGHT], self.encValBuf[RIGHT], self.encPWMBuf[RIGHT], self.encNNewBuf[RIGHT]]))
        s = [[str(e) for e in row] for row in matrix]
        lens = [len(max(col, key=len)) for col in zip(*s)]
        fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
        table = [fmt.format(*row) for row in s]
        f = open('output.txt','w')
        f.write('\n'.join(table))
        f.close()
        print "Wrote buffer to output.txt"
            

class encoderRead(threading.Thread):
    """The encoderRead Class"""
    
    # === Class Properties ===
    # Parameters
    
    # === Class Methods ===
    # Constructor
    def __init__(self,encPin=('P9_39', 'P9_37')):
        
        # Initialize thread
        threading.Thread.__init__(self)
        
        # Set properties
        self.encPin = encPin
        
    # Methods
    def run(self):
        global RUN_FLAG
        
        self.t0 = time.time()
        while RUN_FLAG:
            global ENC_IND
            global ENC_TIME
            global ENC_VAL
            
            for side in range(0,2):
                ENC_TIME[side][ENC_IND[side]] = time.time() - self.t0
                ADC_LOCK.acquire()
                ENC_VAL[side][ENC_IND[side]] = ADC.read_raw(self.encPin[side])
                time.sleep(ADCTIME)
                ADC_LOCK.release()
                ENC_IND[side] = (ENC_IND[side] + 1) % ENC_BUF_SIZE

            
