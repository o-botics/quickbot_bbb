#!/usr/bin/python
"""
@brief QuickBot class for Beaglebone Black

@author Rowland O'Flaherty (rowlandoflaherty.com)
@date 05/30/2014
@version: 2.0
@copyright: Copyright (C) 2014, Georgia Tech Research Corporation
see the LICENSE file included with this software (see LINENSE file)
"""

from __future__ import division
import threading
import time
import base
import utils
import re
import serial
import math

import numpy as np
import quickbot_v2_config as config
import Adafruit_BBIO.ADC as ADC

from base import RIGHT
from base import LEFT

# ADC Constants
ADCTIME = 0.002
ADC_LOCK = threading.Lock()

## Run variables
RUN_FLAG = True
RUN_FLAG_LOCK = threading.Lock()


class QuickBot(base.BaseBot):
    """The QuickBot Class"""

    # Parameters
    sampleTime = 20.0 / 1000.0

    # Motor Pins -- (LEFT, RIGHT)
    dir1Pin = (config.INl1, config.INr1)
    dir2Pin = (config.INl2, config.INr2)
    pwmPin = (config.PWMl, config.PWMr)

    # LED pin
    led = config.LED

    # Encoder Serial
    # TTYO1: Rx=P9_26  Tx=P9_24
    # TTYO2: Rx=P9_22  Tx=P9_21
    # TTYO4: Rx=P9_11  Tx=P9_13
    # TTYO5: Rx=P9_38  Tx=P9_38
    encoderSerial = (serial.Serial(port = '/dev/ttyO1', baudrate = 38400, timeout = .1), \
                     serial.Serial(port = '/dev/ttyO2', baudrate = 38400, timeout = .1))
    encoderBuffer = ['', '']

    # Constraints
    pwmLimits = [-100, 100]  # [min, max]

    # Wheel parameter
    ticksPerTurn = 128  # Number of ticks on encoder disc
    wheelRadius = (58.7 / 2.0) / 1000.0  # Radius of wheel in meters

    # State PWM -- (LEFT, RIGHT)
    pwm = [0, 0]

    # State Encoder
    encTime = [0.0, 0.0]  # Last time encoders were read
    encPos = [0.0, 0.0]  # Last encoder tick position
    encVel = [0.0, 0.0]  # Last encoder tick velocity

    def __init__(self, baseIP, robotIP):
        super(QuickBot, self).__init__(baseIP, robotIP)

        # State IR
        self.nIR = len(self.IRPin)
        self.IRVal = self.nIR*[0.0]

        # State Encoder
        self.encDir = [1, -1]      # Last encoder direction
        self.encPos = [0, 0]      # Last encoder tick position
        self.encVel = [0.0, 0.0]  # Last encoder tick velocity
        self.encPosOffset = [0, 0]  # Offset from raw encoder tick

        # Initialize ADC
        ADC.setup()

        # Initialize IR thread
        self.IRThread = threading.Thread(target=readIR, args=(self, ))
        self.IRThread.daemon = True

        # Initialize encoder threads
        self.encDirThread = 2*[None]
        self.encPosThread = 2*[None]
        self.encVelThread = 2*[None]
        for side in range(0, 2):
            self.encPosThread[side] = threading.Thread(
                target=readEncPos, args=(self, side))
            self.encPosThread[side].daemon = True

        # Calibrate encoders
        self.calibrateEncPos()

    def startThreads(self):
        self.IRThread.start()
        for side in range(0, 2):
            self.encPosThread[side].start()

    def calibrateEncPos(self):
        self.setPWM([100, 100])
        time.sleep(0.1)
        self.setPWM([0, 0])
        time.sleep(1.0)
        self.resetEncPos()

    def getEncPos(self):
        return [self.encPos[LEFT] - self.encPosOffset[LEFT],
                -1*(self.encPos[RIGHT] - self.encPosOffset[RIGHT])]

    def getPos(self):
        pos = [0.0, 0.0]
        encPos = self.getEncPos()
        for side in range(0, 2):
            pos[side] = encPos[side] / self.ticksPerTurn * \
                2 * np.pi * self.wheelRadius
        return pos

    def resetEncPos(self):
        self.encPosOffset[LEFT] = self.encPos[LEFT]
        self.encPosOffset[RIGHT] = self.encPos[RIGHT]

    def update(self):
        pass

    def readIRValues(self):
        prevVal = self.irVal[self.ithIR]
        ADC_LOCK.acquire()
        self.irVal[self.ithIR] = ADC.read_raw(config.IRS[self.ithIR])
        time.sleep(ADCTIME)
        ADC_LOCK.release()

        if self.irVal[self.ithIR] >= 1100:
            self.irVal[self.ithIR] = prevVal

        self.ithIR = ((self.ithIR + 1) % 5)


def readIR(self):
    global RUN_FLAG

    while RUN_FLAG:
        for i in range(0, self.nIR):
            ADC_LOCK.acquire()
            self.IRVal[i] = ADC.read_raw(self.IRPin[i])
            time.sleep(ADCTIME)
            ADC_LOCK.release()


def readEncPos(self, side):
    global RUN_FLAG
    sampleTime = (20.0 / 1000.0)

    while RUN_FLAG:
        parseEncoderBuffer(self, side)
        time.sleep(sampleTime)


def parseEncoderBuffer(self, side):
    encoderUpdateFlag = False

    bytesInWaiting = self.encoderSerial[side].inWaiting()

    if (bytesInWaiting > 0):
        self.encoderBuffer[side] += \
            self.encoderSerial[side].read(bytesInWaiting)

        if len(self.encoderBuffer[side]) > 30:
            self.encoderBuffer[side] = self.encoderBuffer[side][-30:]

        if len(self.encoderBuffer[side]) >= 15:
            DPattern = r'D([0-9A-F]{8})'
            DRegex = re.compile(DPattern)
            DResult = DRegex.findall(self.encoderBuffer[side])
            if len(DResult) >= 1:
                val = utils.convertHEXtoDEC(DResult[-1], 8)
                if not math.isnan(val):
                    self.encPos[side] = val
                    encoderUpdateFlag = True

            VPattern = r'V([0-9A-F]{4})'
            VRegex = re.compile(VPattern)
            VResult = VRegex.findall(self.encoderBuffer[side])
            if len(VResult) >= 1:
                vel = utils.convertHEXtoDEC(VResult[-1], 4)
                if not math.isnan(vel):
                    self.encVel[side] = vel
                    encoderUpdateFlag = True

        return encoderUpdateFlag
