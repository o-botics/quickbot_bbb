"""
@brief QuickBot class

@author Rowland O'Flaherty (rowlandoflaherty.com)
@date 05/30/2014
@version: 2.0
@copyright: Copyright (C) 2014, see the LICENSE file
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

class QuickBot(base.BaseBot):
    """The QuickBot Class"""

    # Parameters
    sample_time = 20.0 / 1000.0

    # Motor Pins -- (LEFT, RIGHT)
    dir1Pin = (config.MOTOR_L.dir1, config.MOTOR_R.dir1)
    dir2Pin = (config.MOTOR_L.dir2, config.MOTOR_R.dir2)
    pwmPin = (config.MOTOR_L.pwm, config.MOTOR_R.pwm)

    # LED pin
    led = config.LED

    # Encoder Serial
    encoderSerial = (
        serial.Serial(
            port=config.EN_L.port,
            baudrate=config.EN_L.baudrate,
            timeout=.1),
        serial.Serial(
            port=config.EN_R.port,
            baudrate=config.EN_R.baudrate,
            timeout=.1))
    encoderBuffer = ['', '']

    # Wheel parameter
    ticksPerTurn = 128  # Number of ticks on encoder disc
    wheelRadius = (58.7 / 2.0) / 1000.0  # Radius of wheel in meters

    # State Encoder
    encTime = [0.0, 0.0]  # Last time encoders were read
    encRaw = [0.0, 0.0]  # Last encoder tick position
    encVel = [0.0, 0.0]  # Last encoder tick velocity

    def __init__(self, base_ip, robot_ip):
        super(QuickBot, self).__init__(base_ip, robot_ip)

        # State IR
        self.nIR = len(config.IR)
        self.irVal = self.nIR*[0.0]

        # State Encoder
        self.encDir = [1, -1]      # Last encoder direction
        self.encRaw = [0, 0]      # Last encoder tick position
        self.encVel = [0.0, 0.0]  # Last encoder tick velocity
        self.encOffset = [0, 0]  # Offset from raw encoder tick

        # Initialize ADC
        ADC.setup()

        # Initialize IR thread
        self.irThread = threading.Thread(target=readIR, args=(self, ))
        self.irThread.daemon = True

        # Initialize encoder threads
        self.encDirThread = 2*[None]
        self.encPosThread = 2*[None]
        self.encVelThread = 2*[None]
        for side in range(0, 2):
            self.encPosThread[side] = threading.Thread(
                target=readEncPos, args=(self, side))
            self.encPosThread[side].daemon = True

    def start_threads(self):
        self.irThread.start()
        for side in range(0, 2):
            self.encPosThread[side].start()

        # Calibrate encoders
        self.calibrateEncPos()

        # Call parent method
        super(QuickBot, self).start_threads()


    def getIr(self):
        return self.irVal

    def calibrateEncPos(self):
        self.set_pwm([100, 100])
        time.sleep(0.1)
        self.set_pwm([0, 0])
        time.sleep(1.0)
        self.resetEncPos()

    def getEncRaw(self):
        return self.encRaw

    def getEncPos(self):
        return [self.encRaw[LEFT] - self.encOffset[LEFT],
                -1*(self.encRaw[RIGHT] - self.encOffset[RIGHT])]

    def getPos(self):
        pos = [0.0, 0.0]
        encPos = self.getEncPos()
        for side in range(0, 2):
            pos[side] = encPos[side] / self.ticksPerTurn * \
                2 * np.pi * self.wheelRadius
        return pos

    def getEncOffset(self):
        return self.encOffset

    def resetEncPos(self):
        self.encOffset[LEFT] = self.encRaw[LEFT]
        self.encOffset[RIGHT] = self.encRaw[RIGHT]

    def getEncVel(self):
        return self.encVel


def readIR(self):
    while self.run_flag:
        for i in range(0, self.nIR):
            ADC_LOCK.acquire()
            self.irVal[i] = ADC.read_raw(config.IR[i])
            time.sleep(ADCTIME)
            ADC_LOCK.release()


def readEncPos(self, side):
    while self.run_flag:
        parseEncoderBuffer(self, side)
        time.sleep(self.sample_time)


def parseEncoderBuffer(self, side):
    encoderUpdateFlag = False

    bytesInWaiting = self.encoderSerial[side].inWaiting()

    if bytesInWaiting > 0:
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
                    self.encRaw[side] = val
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
