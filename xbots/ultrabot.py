"""
@brief UltraBot class for Beaglebone Black

@author Josip Delic (delijati.net)
@date 04/23/2014
@version: 0.1
@copyright: Copyright (C) 2014, see the LICENSE file
"""

from __future__ import division
import threading
import time
import base
import math

import ultrabot_config as config
import Adafruit_BBIO.GPIO as GPIO

# trigger duration
DECPULSETRIGGER = 0.0001
# loop iterations before timeout called
INTTIMEOUT = 2100
MAX_CENTIMETER = 350

# encoder
WHEEL_RADIUS = 1.2  # cm
TIME_INTERVAL = 1.0  # seconds
TICKS = 20  # holes in disc
CONST = (2 * math.pi * WHEEL_RADIUS)/TICKS

# globals
ENC_VEL = [0.0, 0.0]
ENC_POS = [0.0, 0.0]


class UltraBot(base.BaseBot):
    """The UltraBot Class"""
    # Motor Pins -- (LEFT, RIGHT)
    dir1Pin = (config.INl1, config.INr1)
    dir2Pin = (config.INl2, config.INr2)
    pwmPin = (config.PWMl, config.PWMr)
    pwm_freq = 100

    # LED pin
    led = config.LED

    # Constraints
    pwmLimits = [-100, 100]  # [min, max]

    # State PWM -- (LEFT, RIGHT)
    pwm = [0, 0]

    # State Ultras
    ultraVal = [0.0, 0.0, 0.0, 0.0, 0.0]

    # State ir
    irVal = [0, 0, 0, 0, 0]

    # State Encoder
    encPos = [0.0, 0.0]  # Last encoder tick position
    encVel = [0.0, 0.0]  # Last encoder tick velocity

    def __init__(self, baseIP, robotIP):
        super(UltraBot, self).__init__(baseIP, robotIP)
        # init encoder
        self.encoderRead = EncoderReader()
        # init ultras
        self._setup_ultras()

    def update(self):
        self.read_ultras()
        self.read_encoders()
        self.parseCmdBuffer()

    def _setup_ultras(self):
        for trigger, echo in config.ULTRAS:
            GPIO.setup(echo, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(trigger, GPIO.OUT)
            GPIO.output(trigger, False)

    def _measure_ultra(self, trigger, echo):
        GPIO.output(trigger, True)
        time.sleep(DECPULSETRIGGER)
        GPIO.output(trigger, False)

        # Wait for echo to go high (or timeout)
        intcountdown = INTTIMEOUT

        while (GPIO.input(echo) == 0 and intcountdown > 0):
            intcountdown = intcountdown - 1

        # If echo is high
        if intcountdown > 0:

            # Start timer and init timeout countdown
            echostart = time.time()
            intcountdown = INTTIMEOUT

            # Wait for echo to go low (or timeout)
            while (GPIO.input(echo) == 1 and intcountdown > 0):
                intcountdown = intcountdown - 1

            # Stop timer
            echoend = time.time()

            # Echo duration
            echoduration = echoend - echostart

        # Display distance
        if intcountdown > 0:
            intdistance = (echoduration*1000000)/58
            #print "Distance = " + str(intdistance) + "cm"
            return intdistance

    def read_ultras(self):
        for idx, (trigger, echo) in enumerate(config.ULTRAS):
            prevVal = self.ultraVal[idx]
            distance = self._measure_ultra(trigger, echo)
            self.ultraVal[idx] = distance

            if self.ultraVal[idx] >= MAX_CENTIMETER or distance is None:
                self.ultraVal[idx] = prevVal

    def read_encoders(self):
        self.encPos = ENC_POS  # New tick count
        self.encVel = ENC_VEL  # New tick velocity


class EncoderReader(threading.Thread):
    """EncoderReader thread"""

    counter_l = 0
    counter_r = 0

    def __init__(self, encPin=(config.Ol, config.Or)):
        GPIO.setup(config.Ol, GPIO.IN)
        GPIO.setup(config.Or, GPIO.IN)

        # Initialize thread
        threading.Thread.__init__(self)

    def update_encoder_l(self, channel):
        global ENC_POS
        self.counter_l = self.counter_l + 1
        ENC_POS[base.LEFT] = self.counter_l
        #print "Encoder (left) counter updated: %d" % self.counter_l

    def update_encoder_r(self, channel):
        global ENC_POS
        self.counter_r = self.counter_r + 1
        ENC_POS[base.RIGHT] = self.counter_l
        #print "Encoder (right) counter updated: %d" % self.counter_r

    def run(self):
        GPIO.add_event_detect(config.Ol, GPIO.RISING,
                              callback=self.update_encoder_l)
        GPIO.add_event_detect(config.Or, GPIO.RISING,
                              callback=self.update_encoder_r)

        current_time_l = time.time()
        current_time_r = time.time()
        while base.RUN_FLAG:
            if (time.time() >= current_time_l + TIME_INTERVAL):
                global ENC_VEL
                velocity_l = (
                    self.counter_l * (WHEEL_RADIUS * CONST)
                ) / TIME_INTERVAL
                ENC_VEL[base.LEFT] = velocity_l

                self.counter_l = 0
                current_time_l = time.time()
                #print "velocity_l %s cm/s" % velocity_l
            if (time.time() >= current_time_r + TIME_INTERVAL):
                global ENC_VEL
                velocity_r = (
                    self.counter_r * (WHEEL_RADIUS * CONST)
                ) / TIME_INTERVAL
                ENC_VEL[base.RIGHT] = velocity_r

                self.counter_r = 0
                current_time_r = time.time()
                #print "velocity_r %s cm/s" % velocity_r
