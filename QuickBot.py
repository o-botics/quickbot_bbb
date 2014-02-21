#!/usr/bin/python
"""
@brief QuickBot class for Beaglebone Black

@author Rowland O'Flaherty (rowlandoflaherty.com)
@date 02/07/2014
@version: 1.0
@copyright: Copyright (C) 2014, Georgia Tech Research Corporation
see the LICENSE file included with this software (see LINENSE file)
"""

from __future__ import division
import sys
import time
import re
import socket
import threading
import numpy as np

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC

# Constants
LEFT = 0
RIGHT = 1
MIN = 0
MAX = 1

DEBUG = False

ADCTIME = 0.001

## Tic toc constants
TICTOC_START = 0
TICTOC_COUNT = 0
TICTOC_MEAN = 0
TICTOC_MAX = -float('inf')
TICTOC_MIN = float('inf')

## Encoder buffer constants and variables
ENC_BUF_SIZE = 2**9
ENC_IND = [0, 0]
ENC_TIME = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]
ENC_VAL = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]

ADC_LOCK = threading.Lock()

## Run variables
RUN_FLAG = True
RUN_FLAG_LOCK = threading.Lock()


class QuickBot():
    """The QuickBot Class"""

    # === Class Properties ===
    # Parameters
    sampleTime = 20.0 / 1000.0

    # Pins
    ledPin = 'USR1'

    # Motor Pins -- (LEFT, RIGHT)
    dir1Pin = ('P8_14', 'P8_12')
    dir2Pin = ('P8_16', 'P8_10')
    pwmPin = ('P9_16', 'P9_14')

    # ADC Pins
    irPin = ('P9_38', 'P9_40', 'P9_36', 'P9_35', 'P9_33')
    encoderPin = ('P9_39', 'P9_37')

    # Encoder counting parameter and variables
    ticksPerTurn = 16  # Number of ticks on encoder disc
    encWinSize = 2**5  # Should be power of 2
    minPWMThreshold = [45, 45]  # Threshold on the minimum value to turn wheel
    encTPrev = [0.0, 0.0]
    encThreshold = [0.0, 0.0]
    encTickState = [0, 0]
    encTickStateVec = np.zeros((2, encWinSize))

    # Constraints
    pwmLimits = [-100, 100]  # [min, max]

    # State PWM -- (LEFT, RIGHT)
    pwm = [0, 0]

    # State IR
    irVal = [0.0, 0.0, 0.0, 0.0, 0.0]
    ithIR = 0

    # State Encoder
    encTime = [0.0, 0.0]  # Last time encoders were read
    encPos = [0.0, 0.0]  # Last encoder tick position
    encVel = [0.0, 0.0]  # Last encoder tick velocity

    # Encoder counting parameters
    encCnt = 0  # Count of number times encoders have been read
    encSumN = [0, 0]  # Sum of total encoder samples
    encBufInd0 = [0, 0]  # Index of beginning of new samples in buffer
    encBufInd1 = [0, 0]  # Index of end of new samples in buffer
    encTimeWin = np.zeros((2, encWinSize))  # Moving window of encoder sample times
    encValWin = np.zeros((2, encWinSize))  # Moving window of encoder raw sample values
    encPWMWin = np.zeros((2, encWinSize))  # Moving window corresponding PWM input values
    encTau = [0.0, 0.0]  # Average sampling time of encoders

    ## Stats of encoder values while input = 0 and vel = 0
    encZeroCntMin = 2**4  # Min number of recorded values to start calculating stats
    encZeroMean = [0.0, 0.0]
    encZeroVar = [0.0, 0.0]
    encZeroCnt = [0, 0]
    encHighCnt = [0, 0]
    encLowCnt = [0, 0]
    encLowCntMin = 2

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

        
        if DEBUG:
            ## Stats of encoder values while moving -- high, low, and all tick state
            self.encHighLowCntMin = 2**5  # Min number of recorded values to start calculating stats
            self.encHighMean = [0.0, 0.0]
            self.encHighVar = [0.0, 0.0]
            self.encHighTotalCnt = [0, 0]
        
            self.encLowMean = [0.0, 0.0]
            self.encLowVar = [0.0, 0.0]
            self.encLowTotalCnt = [0, 0]
        
            self.encNonZeroCntMin = 2**5
            self.encNonZeroMean = [0.0, 0.0]
            self.encNonZeroVar = [0.0, 0.0]
            self.encNonZeroCnt = [0, 0]

            # Record variables
            self.encRecSize = 2**13
            self.encRecInd = [0, 0]
            self.encTimeRec = np.zeros((2, self.encRecSize))
            self.encValRec = np.zeros((2, self.encRecSize))
            self.encPWMRec = np.zeros((2, self.encRecSize))
            self.encNNewRec = np.zeros((2, self.encRecSize))
            self.encPosRec = np.zeros((2, self.encRecSize))
            self.encVelRec = np.zeros((2, self.encRecSize))
            self.encTickStateRec = np.zeros((2, self.encRecSize))
            self.encThresholdRec = np.zeros((2, self.encRecSize))

    # Getters and Setters
    def setPWM(self, pwm):
        # [leftSpeed, rightSpeed]: 0 is off, caps at min and max values

        self.pwm[LEFT] = min(
            max(pwm[LEFT], self.pwmLimits[MIN]), self.pwmLimits[MAX])
        self.pwm[RIGHT] = min(
            max(pwm[RIGHT], self.pwmLimits[MIN]), self.pwmLimits[MAX])

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
        try:
            while RUN_FLAG is True:
                self.update()

                # Flash BBB LED
                if self.ledFlag is True:
                    self.ledFlag = False
                    GPIO.output(self.ledPin, GPIO.HIGH)
                else:
                    self.ledFlag = True
                    GPIO.output(self.ledPin, GPIO.LOW)
                time.sleep(self.sampleTime)
        except:
            RUN_FLAG_LOCK.acquire()
            RUN_FLAG = False
            RUN_FLAG_LOCK.release()
            raise

        self.cleanup()
        return

    def cleanup(self):
        sys.stdout.write("Shutting down...")
        self.setPWM([0, 0])
        self.robotSocket.close()
        GPIO.cleanup()
        PWM.cleanup()
        if DEBUG:
            # tictocPrint()
            self.writeBuffersToFile()
        sys.stdout.write("Done\n")

    def update(self):
        self.readIRValues()
        self.readEncoderValues()
        self.parseCmdBuffer()

    def parseCmdBuffer(self):
        global RUN_FLAG
        try:
            line = self.robotSocket.recv(1024)
        except socket.error as msg:
            return

        self.cmdBuffer += line

        bufferPattern = r'\$[^\$\*]*?\*'  # String contained within $ and * symbols with no $ or * symbols in it
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
                        pwm = [int(pwmRegex.match(args).group('LEFT')),
                            int(pwmRegex.match(args).group('RIGHT'))]
                        self.setPWM(pwm)

            elif msgResult.group('CMD') == 'IRVAL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.irVal)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))

            elif msgResult.group('CMD') == 'ENVAL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.encPos)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))

            elif msgResult.group('CMD') == 'ENVEL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.encVel)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))

            elif msgResult.group('CMD') == 'RESET':
                self.encPos[LEFT] = 0.0
                self.encPos[RIGHT] = 0.0
                print 'Encoder values reset to [' + ', '.join(map(str, self.encVel)) + ']'

            elif msgResult.group('CMD') == 'UPDATE':
                if msgResult.group('SET') and msgResult.group('ARGS'):
                    args = msgResult.group('ARGS')
                    pwmArgPattern = r'(?P<LEFT>[-]?\d+),(?P<RIGHT>[-]?\d+)'
                    pwmRegex = re.compile(pwmArgPattern)
                    pwmResult = pwmRegex.match(args)
                    if pwmResult:
                        pwm = [int(pwmRegex.match(args).group('LEFT')),
                            int(pwmRegex.match(args).group('RIGHT'))]
                        self.setPWM(pwm)

                    reply = '[' + ', '.join(map(str, self.encPos)) + ', ' \
                        + ', '.join(map(str, self.encVel)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))

            elif msgResult.group('CMD') == 'END':
                RUN_FLAG_LOCK.acquire()
                RUN_FLAG = False
                RUN_FLAG_LOCK.release()

    def readIRValues(self):
        prevVal = self.irVal[self.ithIR]
        ADC_LOCK.acquire()
        self.irVal[self.ithIR] = ADC.read_raw(self.irPin[self.ithIR])
        time.sleep(ADCTIME)
        ADC_LOCK.release()

        if self.irVal[self.ithIR] >= 1100:
                self.irVal[self.ithIR] = prevVal

        self.ithIR = ((self.ithIR+1) % 5)


    def readEncoderValues(self):
        if DEBUG and (self.encCnt % 10) == 0:
            print "------------------------------------"
            print "EncPos:    " + str(self.encPos)
            print "EncVel:    " + str(self.encVel)
            print "***"
            print "Threshold: " + str(self.encThreshold)
            print "***"
            print "Zero Cnt:  " + str(self.encZeroCnt)
            print "Zero Mean: " + str(self.encZeroMean)
            print "Zero Var:  " + str(self.encZeroVar)
            print "***"
            print "NonZero Cnt:  " + str(self.encNonZeroCnt)
            print "NonZero Mean: " + str(self.encNonZeroMean)
            print "NonZero Var:  " + str(self.encNonZeroVar)
            print "***"
            print "High Cnt:  " + str(self.encHighTotalCnt)
            print "High Mean: " + str(self.encHighMean)
            print "High Var:  " + str(self.encHighVar)
            print "***"
            print "Low Cnt:  " + str(self.encLowTotalCnt)
            print "Low Mean: " + str(self.encLowMean)
            print "Low Var:  " + str(self.encLowVar)

        self.encCnt = self.encCnt + 1

        # Fill window
        for side in range(0, 2):
            self.encTime[side] = self.encTimeWin[side][-1]

            self.encBufInd0[side] = self.encBufInd1[side]
            self.encBufInd1[side] = ENC_IND[side]
            ind0 = self.encBufInd0[side]  # starting index
            ind1 = self.encBufInd1[side]  # ending index (this element is not included until the next update)

            if ind0 < ind1:
                N = ind1 - ind0  # number of new elements
                self.encSumN[side] = self.encSumN[side] + N
                self.encTimeWin[side] = np.roll(self.encTimeWin[side], -N)
                self.encTimeWin[side, -N:] = ENC_TIME[side][ind0:ind1]
                self.encValWin[side] = np.roll(self.encValWin[side], -N)
                self.encValWin[side, -N:] = ENC_VAL[side][ind0:ind1]
                self.encPWMWin[side] = np.roll(self.encPWMWin[side], -N)
                self.encPWMWin[side, -N:] = [self.pwm[side]]*N

            elif ind0 > ind1:
                N = ENC_BUF_SIZE - ind0 + ind1  # number of new elements
                self.encSumN[side] = self.encSumN[side] + N
                self.encTimeWin[side] = np.roll(self.encTimeWin[side], -N)
                self.encValWin[side] = np.roll(self.encValWin[side], -N)
                self.encPWMWin[side] = np.roll(self.encPWMWin[side], -N)
                self.encPWMWin[side, -N:] = [self.pwm[side]]*N
                if ind1 == 0:
                    self.encTimeWin[side, -N:] = ENC_TIME[side][ind0:]
                    self.encValWin[side, -N:] = ENC_VAL[side][ind0:]
                else:
                    self.encTimeWin[side, -N:-ind1] = ENC_TIME[side][ind0:]
                    self.encValWin[side, -N:-ind1] = ENC_VAL[side][ind0:]
                    self.encTimeWin[side, -ind1:] = ENC_TIME[side][0:ind1]
                    self.encValWin[side, -ind1:] = ENC_VAL[side][0:ind1]

            if ind0 != ind1:
                tauNew = self.encTimeWin[side,-1] - self.encTimeWin[side,-N]
                self.encTau[side] = tauNew / self.encCnt + self.encTau[side] * (self.encCnt-1)/self.encCnt  # Running average
                if self.encSumN[side] > self.encWinSize:
                    self.countEncoderTicks(side)

                # Fill records
                if DEBUG:
                    ind = self.encRecInd[side]
                    if ind+N < self.encRecSize:
                        self.encTimeRec[side, ind:ind+N] = self.encTimeWin[side, -N:]
                        self.encValRec[side, ind:ind+N] = self.encValWin[side, -N:]
                        self.encPWMRec[side, ind:ind+N] = self.encPWMWin[side, -N:]
                        self.encNNewRec[side, ind:ind+N] = [N]*N
                        self.encPosRec[side, ind:ind+N] = [self.encPos[side]]*N
                        self.encVelRec[side, ind:ind+N] = [self.encVel[side]]*N
                        self.encTickStateRec[side, ind:ind+N] = self.encTickStateVec[side, -N:]
                        self.encThresholdRec[side, ind:ind+N] = [self.encThreshold[side]]*N
                    self.encRecInd[side] = ind+N

    def countEncoderTicks(self, side):
        # Set variables
        t = self.encTimeWin[side]  # Time vector of data (non-consistent sampling time)
        tPrev = self.encTPrev[side]  # Previous read time
        pwm = self.encPWMWin[side]  # Vector of PWM data
        pwmPrev = pwm[-1]  # Last PWM value that was applied
        tickStatePrev = self.encTickState[side]  # Last state of tick (high (1), low (-1), or unsure (0))
        tickCnt = self.encPos[side]  # Current tick count
        tickVel = self.encVel[side]  # Current tick velocity
        encValWin = self.encValWin[side]  # Encoder raw value buffer window
        threshold = self.encThreshold[side]  # Encoder value threshold
        minPWMThreshold = self.minPWMThreshold[side]  # Minimum PWM to move wheel

        N = np.sum(t > tPrev)  # Number of new updates

        tickStateVec = np.roll(self.encTickStateVec[side], -N)

        # Determine wheel direction
        if tickVel != 0:
            wheelDir = np.sign(tickVel)
        else:
            wheelDir = np.sign(pwmPrev)

        # Count ticks and record tick state
        indTuple = np.where(t == tPrev)  # Index of previous sample in window
        if len(indTuple[0] > 0):
            ind = indTuple[0][0]
            newInds = ind + np.arange(1, N+1)  # Indices of new samples
            for i in newInds:
                if encValWin[i] > threshold:  # High tick state
                    tickState = 1
                    self.encHighCnt[side] = self.encHighCnt[side] + 1
                    self.encLowCnt[side] = 0
                    if tickStatePrev == -1:  # Increment tick count on rising edge
                        tickCnt = tickCnt + wheelDir

                else:  # Low tick state
                    tickState = -1
                    self.encLowCnt[side] = self.encLowCnt[side] + 1
                    self.encHighCnt[side] = 0
                tickStatePrev = tickState
                tickStateVec[i] = tickState

            # Measure tick speed
            diffTickStateVec = np.diff(tickStateVec)  # Tick state transition differences
            fallingTimes = t[np.hstack((False,diffTickStateVec == -2))]  # Times when tick state goes from high to low
            risingTimes = t[np.hstack((False,diffTickStateVec == 2))]  # Times when tick state goes from low to high
            fallingPeriods = np.diff(fallingTimes)  # Period times between falling edges
            risingPeriods = np.diff(risingTimes)  # Period times between rising edges
            tickPeriods = np.hstack((fallingPeriods, risingPeriods)) # All period times
            if len(tickPeriods) == 0:
                if all(pwm[newInds] < minPWMThreshold):  # If all inputs are less than min set velocity to 0
                    tickVel = 0
            else:
                tickVel = wheelDir * 1/np.mean(tickPeriods)  # Average signed tick frequency

            # Estimate new mean values
            newEncRaw = encValWin[newInds]
            if pwmPrev == 0 and tickVel == 0:
                x = newEncRaw
                l = self.encZeroCnt[side]
                mu = self.encZeroMean[side]
                sigma2 = self.encZeroVar[side]
                (muPlus, sigma2Plus, n) = recursiveMeanVar(x, l, mu, sigma2)
                self.encZeroMean[side] = muPlus
                self.encZeroVar[side] = sigma2Plus
                self.encZeroCnt[side] = n
            elif tickVel != 0:
                if DEBUG:
                    x = newEncRaw
                    l = self.encNonZeroCnt[side]
                    mu = self.encNonZeroMean[side]
                    sigma2 = self.encNonZeroVar[side]
                    (muPlus, sigma2Plus, n) = recursiveMeanVar(x, l, mu, sigma2)
                    self.encNonZeroMean[side] = muPlus
                    self.encNonZeroVar[side] = sigma2Plus
                    self.encNonZeroCnt[side] = n

                    NHigh = np.sum(tickStateVec[newInds] == 1)
                    if NHigh != 0:
                        indHighTuple = np.where(tickStateVec[newInds] == 1)
                        x = newEncRaw[indHighTuple[0]]
                        l = self.encHighTotalCnt[side]
                        mu = self.encHighMean[side]
                        sigma2 = self.encHighVar[side]
                        (muPlus, sigma2Plus, n) = recursiveMeanVar(x, l, mu, sigma2)
                        self.encHighMean[side] = muPlus
                        self.encHighVar[side] = sigma2Plus
                        self.encHighTotalCnt[side] = n

                    NLow = np.sum(tickStateVec[newInds] == -1)
                    if NLow != 0:
                        indLowTuple = np.where(tickStateVec[newInds] == -1)
                        x = newEncRaw[indLowTuple[0]]
                        l = self.encLowTotalCnt[side]
                        mu = self.encLowMean[side]
                        sigma2 = self.encLowVar[side]
                        (muPlus, sigma2Plus, n) = recursiveMeanVar(x, l, mu, sigma2)
                        self.encLowMean[side] = muPlus
                        self.encLowVar[side] = sigma2Plus
                        self.encLowTotalCnt[side] = n

            # Set threshold value
            if self.encZeroCnt[side] > self.encZeroCntMin:
                self.encThreshold[side] = self.encZeroMean[side] - 3*np.sqrt(self.encZeroVar[side])

#             elif self.encNonZeroCnt[side] > self.encNonZeroCntMin:
#                 self.encThreshold[side] = self.encNonZeroMean[side]

#             elif self.encHighTotalCnt[side] > self.encHighLowCntMin and self.encLowTotalCnt > self.encHighLowCntMin:
#                 mu1 = self.encHighMean[side]
#                 sigma1 = self.encHighVar[side]
#                 mu2 = self.encLowMean[side]
#                 sigma2 = self.encLowVar[side]
#                 alpha = (sigma1 * np.log(sigma1)) / (sigma2 * np.log(sigma1))
#                 A = (1 - alpha)
#                 B = -2 * (mu1 - alpha*mu2)
#                 C = mu1**2 - alpha * mu2**2
#                 x1 = (-B + np.sqrt(B**2 - 4*A*C)) / (2*A)
#                 x2 = (-B - np.sqrt(B**2 - 4*A*C)) / (2*A)
#                 if x1 < mu1 and x1 > mu2:
#                     self.encThreshold[side] = x1
#                 else:
#                     self.encThreshold[side] = x2


            # Update variables
            self.encPos[side] = tickCnt  # New tick count
            self.encVel[side] = tickVel  # New tick velocity
            self.encTickStateVec[side] = tickStateVec  # New tick state vector

        self.encTPrev[side] = t[-1]  # New latest update time


    def writeBuffersToFile(self):
        matrix = map(list, zip(*[self.encTimeRec[LEFT], self.encValRec[LEFT], self.encPWMRec[LEFT], self.encNNewRec[LEFT], \
                                 self.encTickStateRec[LEFT], self.encPosRec[LEFT], self.encVelRec[LEFT], self.encThresholdRec[LEFT], \
                                 self.encTimeRec[RIGHT], self.encValRec[RIGHT], self.encPWMRec[RIGHT], self.encNNewRec[RIGHT], \
                                 self.encTickStateRec[RIGHT], self.encPosRec[RIGHT], self.encVelRec[RIGHT], self.encThresholdRec[RIGHT]]))
        s = [[str(e) for e in row] for row in matrix]
        lens = [len(max(col, key=len)) for col in zip(*s)]
        fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
        table = [fmt.format(*row) for row in s]
        f = open('output.txt', 'w')
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

            for side in range(0, 2):
                ENC_TIME[side][ENC_IND[side]] = time.time() - self.t0
                ADC_LOCK.acquire()
                ENC_VAL[side][ENC_IND[side]] = ADC.read_raw(self.encPin[side])
                time.sleep(ADCTIME)
                ADC_LOCK.release()
                ENC_IND[side] = (ENC_IND[side] + 1) % ENC_BUF_SIZE


def recursiveMeanVar(x, l, mu, sigma2):
    """
    This function calculates a new mean and variance given
    the current mean "mu", current variance "sigma2", current
    update count "l", and new samples "x"
    """

    m = len(x)
    n = l + m
    muPlus = l / n * mu + m / n * np.mean(x)
    if n > 1:
        sigma2Plus = 1/(n-1) * ((l-1)*sigma2 + (m-1)*np.var(x) + l*(mu - muPlus)**2 + m*(np.mean(x) - muPlus)**2)
    else:
        sigma2Plus = 0

    return (muPlus, sigma2Plus, n)

def operatingPoint(uStar, uStarThreshold):
    """
    This function returns the steady state tick velocity given some PWM input.

    uStar: PWM input.
    uStarThreshold: Threshold on the minimum magnitude of a PWM input value

    returns: omegaStar - steady state tick velocity
    """
    # Matlab code to find beta values
    # X = [40; 80; 100]; % Air Test
    # Y = [0.85; 2.144; 3.5];
    #
    # r = 0.0325; % Wheel radius
    # c = 2*pi*r;
    # X = [  70;   70;   70;   75;   75;   75;   80;   80;   80; 85;     85;   85;   90;   90;   90]; % Ground Test
    # Z = [4.25; 3.95; 4.23; 3.67; 3.53; 3.48; 3.19; 3.08; 2.93; 2.52; 2.59; 2.56; 1.99; 2.02; 2.04]; % Time to go 1 m
    # Y = 1./(Z*c);
    # H = [X ones(size(X))];
    # beta = H \ Y
    # beta = [0.0425, -0.9504] # Air Test Results
    beta = [0.0606, -3.1475] # Ground Test Results

    if np.abs(uStar) <= uStarThreshold:
        omegaStar = 0.0
    elif uStar > 0:
        omegaStar = beta[0]*uStar + beta[1]
    else:
        omegaStar = -1.0*(beta[0]*np.abs(uStar) + beta[1])

    return omegaStar


def kalman(x, P, Phi, H, W, V, z):
    """
    This function returns an optimal expected value of the state and covariance
    error matrix given an update and system parameters.

    x:   Estimate of staet at time t-1.
    P:   Estimate of error covariance matrix at time t-1.
    Phi: Discrete time state tranistion matrix at time t-1.
    H:   Observation model matrix at time t.
    W:   Process noise covariance at time t-1.
    V:   Measurement noise covariance at time t.
    z:   Measurement at time t.

    returns: (x,P) tuple
    x: Updated estimate of state at time t.
    P: Updated estimate of error covariance matrix at time t.

    """
    x_p = Phi*x  # Prediction of setimated state vector
    P_p = Phi*P*Phi + W  # Prediction of error covariance matrix
    S = H*P_p*H + V  # Sum of error variances
    S_inv = 1/S  # Inverse of sum of error variances
    K = P_p*H*S_inv  # Kalman gain
    r = z - H*x_p  # Prediction residual
    w = -K*r  # Process error
    x = x_p - w  # Update estimated state vector
    v = z - H*x  # Measurement error
    if np.isnan(K*V):
        P = P_p
    else:
        P = (1 - K*H)*P_p*(1 - K*H) + K*V*K  # Updated error covariance matrix

    return (x, P)


def tic():
    global TICTOC_START
    TICTOC_START = time.time()


def toc(tictocName = 'toc', printFlag = True):
    global TICTOC_START
    global TICTOC_COUNT
    global TICTOC_MEAN
    global TICTOC_MAX
    global TICTOC_MIN

    tictocTime = time.time() - TICTOC_START
    TICTOC_COUNT = TICTOC_COUNT + 1
    TICTOC_MEAN = tictocTime / TICTOC_COUNT + TICTOC_MEAN * (TICTOC_COUNT-1) / TICTOC_COUNT
    TICTOC_MAX = max(TICTOC_MAX,tictocTime)
    TICTOC_MIN = min(TICTOC_MIN,tictocTime)

    if printFlag:
        print tictocName + " time: " + str(tictocTime)

def tictocPrint():
    global TICTOC_COUNT
    global TICTOC_MEAN
    global TICTOC_MAX
    global TICTOC_MIN

    print "Tic Toc Stats:"
    print "Count = " + str(TICTOC_COUNT)
    print "Mean = " + str(TICTOC_MEAN)
    print "Max = " + str(TICTOC_MAX)
    print "Min = " + str(TICTOC_MIN)


