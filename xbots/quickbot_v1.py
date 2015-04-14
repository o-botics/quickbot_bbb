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
import threading
import time
import base
import utils

import numpy as np
import quickbot_v1_config as config
import Adafruit_BBIO.ADC as ADC

# Constants

ADCTIME = 0.001

# Encoder buffer constants and variables
ENC_BUF_SIZE = 2 ** 9
ENC_IND = [0, 0]
ENC_TIME = [[0] * ENC_BUF_SIZE, [0] * ENC_BUF_SIZE]
ENC_VAL = [[0] * ENC_BUF_SIZE, [0] * ENC_BUF_SIZE]

ADC_LOCK = threading.Lock()


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

    # Encoder counting parameter and variables
    ticksPerTurn = 16  # Number of ticks on encoder disc
    encWinSize = 2 ** 5  # Should be power of 2
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
    # Moving window of encoder sample times
    encTimeWin = np.zeros((2, encWinSize))
    # Moving window of encoder raw sample values
    encValWin = np.zeros((2, encWinSize))
    # Moving window corresponding PWM input values
    encPWMWin = np.zeros((2, encWinSize))
    encTau = [0.0, 0.0]  # Average sampling time of encoders

    # Stats of encoder values while input = 0 and vel = 0
    # Min number of recorded values to start calculating stats
    encZeroCntMin = 2 ** 4
    encZeroMean = [0.0, 0.0]
    encZeroVar = [0.0, 0.0]
    encZeroCnt = [0, 0]
    encHighCnt = [0, 0]
    encLowCnt = [0, 0]
    encLowCntMin = 2

    def __init__(self, baseIP, robotIP):
        super(QuickBot, self).__init__(baseIP, robotIP)
        # init encoder
        self.encoderRead = EncoderReader()
        # Initialize ADC
        ADC.setup()

        if base.DEBUG:
            # Stats of encoder values while moving -- high, low, and all tick
            # state
            # Min number of recorded values to start calculating stats
            self.encHighLowCntMin = 2**5
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



    def update(self):
        self.readIRValues()
        self.readEncoderValues()

    def get_ir(self):
        """ Getter for IR sensor values """
        return self.irVal

    def readIRValues(self):
        prevVal = self.irVal[self.ithIR]
        ADC_LOCK.acquire()
        self.irVal[self.ithIR] = ADC.read_raw(config.IRS[self.ithIR])
        time.sleep(ADCTIME)
        ADC_LOCK.release()

        if self.irVal[self.ithIR] >= 1100:
            self.irVal[self.ithIR] = prevVal

        self.ithIR = ((self.ithIR + 1) % 5)

    def readEncoderValues(self):
        """
        We read the raw adc data and try to determine if we had a rising or
        falling edge. After that we try to calculate how many ticks we got.
        """
        if base.DEBUG and (self.encCnt % 10) == 0:
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
            # ending index (this element is not included until the next update)
            ind1 = self.encBufInd1[side]

            if ind0 < ind1:
                N = ind1 - ind0  # number of new elements
                self.encSumN[side] = self.encSumN[side] + N
                self.encTimeWin[side] = np.roll(self.encTimeWin[side], -N)
                self.encTimeWin[side, -N:] = ENC_TIME[side][ind0:ind1]
                self.encValWin[side] = np.roll(self.encValWin[side], -N)
                self.encValWin[side, -N:] = ENC_VAL[side][ind0:ind1]
                self.encPWMWin[side] = np.roll(self.encPWMWin[side], -N)
                self.encPWMWin[side, -N:] = [self.pwm[side]] * N

            elif ind0 > ind1:
                N = ENC_BUF_SIZE - ind0 + ind1  # number of new elements
                self.encSumN[side] = self.encSumN[side] + N
                self.encTimeWin[side] = np.roll(self.encTimeWin[side], -N)
                self.encValWin[side] = np.roll(self.encValWin[side], -N)
                self.encPWMWin[side] = np.roll(self.encPWMWin[side], -N)
                self.encPWMWin[side, -N:] = [self.pwm[side]] * N
                if ind1 == 0:
                    self.encTimeWin[side, -N:] = ENC_TIME[side][ind0:]
                    self.encValWin[side, -N:] = ENC_VAL[side][ind0:]
                else:
                    self.encTimeWin[side, -N:-ind1] = ENC_TIME[side][ind0:]
                    self.encValWin[side, -N:-ind1] = ENC_VAL[side][ind0:]
                    self.encTimeWin[side, -ind1:] = ENC_TIME[side][0:ind1]
                    self.encValWin[side, -ind1:] = ENC_VAL[side][0:ind1]

            if ind0 != ind1:
                tauNew = self.encTimeWin[side, -1] - self.encTimeWin[side, -N]
                # Running average
                self.encTau[side] = tauNew / self.encCnt + \
                    self.encTau[side] * (self.encCnt - 1) / self.encCnt
                if self.encSumN[side] > self.encWinSize:
                    self.countEncoderTicks(side)

                # Fill records
                if base.DEBUG:
                    ind = self.encRecInd[side]
                    if ind + N < self.encRecSize:
                        self.encTimeRec[
                            side, ind:ind + N] = self.encTimeWin[side, -N:]
                        self.encValRec[
                            side, ind:ind + N] = self.encValWin[side, -N:]
                        self.encPWMRec[
                            side, ind:ind + N] = self.encPWMWin[side, -N:]
                        self.encNNewRec[side, ind:ind + N] = [N] * N
                        self.encPosRec[
                            side, ind:ind + N] = [self.encPos[side]] * N
                        self.encVelRec[
                            side, ind:ind + N] = [self.encVel[side]] * N
                        self.encTickStateRec[
                            side, ind:ind + N] = self.encTickStateVec[side,
                                                                      -N:]
                        self.encThresholdRec[
                            side, ind:ind + N] = [self.encThreshold[side]] * N
                    self.encRecInd[side] = ind + N

    def countEncoderTicks(self, side):
        # Set variables
        # Time vector of data (non-consistent sampling time)
        t = self.encTimeWin[side]
        tPrev = self.encTPrev[side]  # Previous read time
        pwm = self.encPWMWin[side]  # Vector of PWM data
        pwmPrev = pwm[-1]  # Last PWM value that was applied
        # Last state of tick (high (1), low (-1), or unsure (0))
        tickStatePrev = self.encTickState[side]
        tickCnt = self.encPos[side]  # Current tick count
        tickVel = self.encVel[side]  # Current tick velocity
        encValWin = self.encValWin[side]  # Encoder raw value buffer window
        threshold = self.encThreshold[side]  # Encoder value threshold
        # Minimum PWM to move wheel
        minPWMThreshold = self.minPWMThreshold[side]

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
            newInds = ind + np.arange(1, N + 1)  # Indices of new samples
            for i in newInds:
                if encValWin[i] > threshold:  # High tick state
                    tickState = 1
                    self.encHighCnt[side] = self.encHighCnt[side] + 1
                    self.encLowCnt[side] = 0
                    # Increment tick count on rising edge
                    if tickStatePrev == -1:
                        tickCnt = tickCnt + wheelDir

                else:  # Low tick state
                    tickState = -1
                    self.encLowCnt[side] = self.encLowCnt[side] + 1
                    self.encHighCnt[side] = 0
                tickStatePrev = tickState
                tickStateVec[i] = tickState

            # Measure tick speed
            # Tick state transition differences
            diffTickStateVec = np.diff(tickStateVec)
            # Times when tick state goes from high to low
            fallingTimes = t[np.hstack((False, diffTickStateVec == -2))]
            # Times when tick state goes from low to high
            risingTimes = t[np.hstack((False, diffTickStateVec == 2))]
            # Period times between falling edges
            fallingPeriods = np.diff(fallingTimes)
            # Period times between rising edges
            risingPeriods = np.diff(risingTimes)
            tickPeriods = np.hstack(
                (fallingPeriods, risingPeriods))  # All period times
            if len(tickPeriods) == 0:
                # If all inputs are less than min set velocity to 0
                if all(pwm[newInds] < minPWMThreshold):
                    tickVel = 0
            else:
                # Average signed tick frequency
                tickVel = wheelDir * 1 / np.mean(tickPeriods)

            # Estimate new mean values
            newEncRaw = encValWin[newInds]
            if pwmPrev == 0 and tickVel == 0:
                x = newEncRaw
                l = self.encZeroCnt[side]
                mu = self.encZeroMean[side]
                sigma2 = self.encZeroVar[side]
                (muPlus, sigma2Plus, n) = utils.recursiveMeanVar(x, l, mu,
                                                                 sigma2)
                self.encZeroMean[side] = muPlus
                self.encZeroVar[side] = sigma2Plus
                self.encZeroCnt[side] = n
            elif tickVel != 0:
                if base.DEBUG:
                    x = newEncRaw
                    l = self.encNonZeroCnt[side]
                    mu = self.encNonZeroMean[side]
                    sigma2 = self.encNonZeroVar[side]
                    (muPlus, sigma2Plus, n) = utils.recursiveMeanVar(
                        x, l, mu, sigma2)
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
                        (muPlus, sigma2Plus, n) = utils.recursiveMeanVar(
                            x, l, mu, sigma2)
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
                        (muPlus, sigma2Plus, n) = utils.recursiveMeanVar(
                            x, l, mu, sigma2)
                        self.encLowMean[side] = muPlus
                        self.encLowVar[side] = sigma2Plus
                        self.encLowTotalCnt[side] = n

            # Set threshold value
            if self.encZeroCnt[side] > self.encZeroCntMin:
                self.encThreshold[side] = self.encZeroMean[
                    side] - 3 * np.sqrt(self.encZeroVar[side])

#             elif self.encNonZeroCnt[side] > self.encNonZeroCntMin:
#                 self.encThreshold[side] = self.encNonZeroMean[side]

#             elif self.encHighTotalCnt[side] > self.encHighLowCntMin and \
#                    self.encLowTotalCnt > self.encHighLowCntMin:
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


class EncoderReader(threading.Thread):
    """EncoderReader thread"""

    def __init__(self, encPin=(config.Ol, config.Or)):

        # Initialize thread
        threading.Thread.__init__(self)

        # Set properties
        self.encPin = encPin

    def run(self):
        self.t0 = time.time()

        while base.RUN_FLAG:
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
