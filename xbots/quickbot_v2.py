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
from numpy import pi as PI

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

    # Wheel parameters
    ticksPerTurn = 128  # Number of ticks on encoder disc
    wheelRadius = (58.7 / 2.0) / 1000.0  # Radius of wheel in meters

    # Encoder parameters
    enc_vel_buf_size = 10 # Size of encoder velocity buffer

    # State Encoder
    enc_raw = [0.0, 0.0]  # Last encoder tick position
    enc_vel = [0.0, 0.0]  # Last encoder tick velocity

    def __init__(self, base_ip, robot_ip):
        super(QuickBot, self).__init__(base_ip, robot_ip)

        # State IR
        self.n_ir = len(config.IR)
        self.ir_val = self.n_ir*[0.0]

        # State Encoder
        self.enc_dir = [1, -1]      # Last encoder direction
        self.enc_raw = [0, 0]      # Last encoder tick position
        self.enc_vel = [0.0, 0.0]  # Last encoder tick velocity
        self.enc_offset = [0.0, 0.0]  # Offset from raw encoder tick
        self.enc_vel_buf_cnt = [0, 0]  # Encoder velocity buffer counters
        self.enc_vel_buf = [[0.0] * self.enc_vel_buf_size,
                            [0.0] * self.enc_vel_buf_size]  # Encoder velocity buffers


        # Initialize ADC
        ADC.setup()

        # Initialize IR thread
        self.ir_thread = threading.Thread(target=read_ir, args=(self, ))
        self.ir_thread.daemon = True

        # Initialize encoder threads
        self.enc_pos_thread = 2*[None]
        for side in range(0, 2):
            self.enc_pos_thread[side] = threading.Thread(
                target=read_enc_val, args=(self, side))
            self.enc_pos_thread[side].daemon = True

    def start_threads(self):
        """ Start all threads """
        self.ir_thread.start()
        for side in range(0, 2):
            self.enc_pos_thread[side].start()

        # Calibrate encoders
        self.calibrate_enc_val()

        # Call parent method
        super(QuickBot, self).start_threads()


    def get_ir(self):
        """ Getter for IR sensor values """
        return self.ir_val

    def calibrate_enc_val(self):
        """ Calibrate wheel encoder values"""
        self.set_pwm([100, 100])
        time.sleep(0.1)
        self.set_pwm([0, 0])
        time.sleep(1.0)
        self.reset_enc_val()

    def get_enc_raw(self):
        """ Getter for raw encoder values """
        return self.enc_raw

    def get_enc_val(self):
        """ Getter for encoder tick values i.e (raw - offset) """
        return [self.enc_raw[LEFT] - self.enc_offset[LEFT],
                -1*(self.enc_raw[RIGHT] - self.enc_offset[RIGHT])]

    def set_enc_val(self, enc_val):
        """ Setter for encoder tick positions """
        offset = [0.0, 0.0]
        offset[LEFT] = self.enc_raw[LEFT] - enc_val[LEFT]
        offset[RIGHT] = -1*(self.enc_raw[RIGHT] - enc_val[RIGHT])
        self.set_enc_offset(offset)

    def get_wheel_ang(self):
        """ Getter for wheel angles """
        ang = [0.0, 0.0]
        enc_val = self.get_enc_val()
        for side in range(0, 2):
            ang[side] = enc_val[side] / self.ticksPerTurn * 2 * PI
        return ang

    def set_wheel_ang(self, ang):  # FIXME - Should move wheel to that angle
        """ Setter for wheel angles """
        enc_val = [0.0, 0.0]
        for side in range(0, 2):
            enc_val[side] = ang[side] * self.ticksPerTurn / (2 * PI)
        self.set_enc_val(enc_val)

    def get_enc_offset(self):
        """ Getter for encoder offset values """
        return self.enc_offset

    def set_enc_offset(self, offset):
        """ Setter for encoder offset values """
        for side in range(0, 2):
            self.enc_offset[side] = offset[side]

    def reset_enc_val(self):
        """ Reset encoder values to 0 """
        self.enc_offset[LEFT] = self.enc_raw[LEFT]
        self.enc_offset[RIGHT] = self.enc_raw[RIGHT]

    def get_enc_vel(self):
        """ Getter for encoder velocity values """
        return self.enc_vel

    def get_wheel_ang_vel(self):
        """ Getter for wheel angular velocity values """
        ang_vel = [0.0, 0.0]
        enc_vel = self.get_enc_vel()
        for side in range(0, 2):
            ang_vel[side] = enc_vel[side] * (2* PI) / self.ticksPerTurn
        return ang_vel

    def set_wheel_ang_vel(self, ang_vel):
        """ Setter for wheel angular velocity values """
        pass



def read_ir(self):
    """ Thread function for reading IR sensor values """
    while self.run_flag:
        for i in range(0, self.n_ir):
            ADC_LOCK.acquire()
            self.ir_val[i] = ADC.read_raw(config.IR[i])
            time.sleep(ADCTIME)
            ADC_LOCK.release()


def read_enc_val(self, side):
    """ Thread function for reading encoder values """
    while self.run_flag:
        parse_encoder_buffer(self, side)
        time.sleep(self.sample_time)


def parse_encoder_buffer(self, side):
    """ Parses encoder serial data """
    encoder_update_flag = False

    bytes_in_waiting = self.encoderSerial[side].inWaiting()

    if bytes_in_waiting > 0:
        self.encoderBuffer[side] += \
            self.encoderSerial[side].read(bytes_in_waiting)

        if len(self.encoderBuffer[side]) > 30:
            self.encoderBuffer[side] = self.encoderBuffer[side][-30:]

        if len(self.encoderBuffer[side]) >= 15:
            d_pattern = r'D([0-9A-F]{8})'
            d_regex = re.compile(d_pattern)
            d_result = d_regex.findall(self.encoderBuffer[side])
            if len(d_result) >= 1:
                val = utils.convertHEXtoDEC(d_result[-1], 8)
                if not math.isnan(val):
                    self.enc_raw[side] = val
                    encoder_update_flag = True

            v_pattern = r'V([0-9A-F]{4})'
            v_regex = re.compile(v_pattern)
            v_result = v_regex.findall(self.encoderBuffer[side])
            if len(v_result) >= 1:
                vel = utils.convertHEXtoDEC(v_result[-1], 4)
                if not math.isnan(vel):
                    if side == 1:
                        vel = -1*vel
                    self.enc_vel_buf_cnt[side] += 1
                    this_cnt = self.enc_vel_buf_cnt[side]
                    this_size = self.enc_vel_buf_size
                    self.enc_vel_buf[side][this_cnt % this_size] = vel
                    this_vel = np.median(self.enc_vel_buf[side])
                    self.enc_vel[side] = this_vel
                    encoder_update_flag = True
    else:
        self.enc_vel_buf_cnt[side] += 1
        this_cnt = self.enc_vel_buf_cnt[side]
        this_size = self.enc_vel_buf_size
        self.enc_vel_buf[side][this_cnt % this_size] = 0.0
        this_vel = np.median(self.enc_vel_buf[side])
        self.enc_vel[side] = this_vel

        return encoder_update_flag
