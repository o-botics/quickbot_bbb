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

    # State encoder
    enc_raw = [0.0, 0.0]  # Last encoder tick position
    enc_vel = [0.0, 0.0]  # Last encoder tick velocity
    enc_time = [0.0, 0.0]  # Last encoder tick sample time
    enc_state = 2*[np.array([[0.0, 0.0]]).T]  # Last encoder state -- pos, vel, acc
    enc_state_cov = 2*[np.array([[1.0, 0.0],
                      [0.0, 1.0]])]

    # Controller paramerters
    enc_vel_controller_flag = 2*[False]

    def __init__(self, base_ip, robot_ip):
        super(QuickBot, self).__init__(base_ip, robot_ip)

        # Start time
        self.t0 = time.time()

        # State IR
        self.n_ir = len(config.IR)
        self.ir_val = self.n_ir*[0.0]

        # State Encoder
        self.enc_dir = [1, -1]         # Last encoder direction
        self.enc_raw = [0, 0]          # Last encoder tick position
        self.enc_vel = [0.0, 0.0]      # Last encoder tick velocity
        self.enc_offset = [0.0, 0.0]   # Offset from raw encoder tick

        # Set Points
        self.enc_vel_set_point = [0.0, 0.0]

        # Initialize ADC
        ADC.setup()

        # Initialize IR thread
        self.ir_thread = threading.Thread(target=read_ir_thread_fcn, args=(self, ))
        self.ir_thread.daemon = True

        # Initialize encoder threads
        self.enc_pos_thread = 2*[None]
        for side in range(0, 2):
            self.enc_pos_thread[side] = threading.Thread(
                target=read_enc_val_thread_fcn, args=(self, side))
            self.enc_pos_thread[side].daemon = True

        # Initialize wheel controller thread
        self.enc_vel_controller_thread = 2*[None]
        for side in range(0, 2):
            self.enc_vel_controller_thread[side] = threading.Thread(
                target=enc_vel_controller_thread_fcn, args=(self, side))
            self.enc_vel_controller_thread[side].daemon = True


    def start_threads(self):
        """ Start all threads """
        self.ir_thread.start()
        for side in range(0, 2):
            self.enc_pos_thread[side].start()
            self.enc_vel_controller_thread[side].start()

        # Calibrate encoders
        self.calibrate_enc_val()

        # Call parent method
        super(QuickBot, self).start_threads()

    def set_pwm_left(self, pwm_left):
        """ Set left motor PWM value """
        self.enc_vel_controller_flag[LEFT] = False
        super(QuickBot, self).set_pwm_left(pwm_left)


    def set_pwm_right(self, pwm_right):
        """ Set right motor PWM value """
        self.enc_vel_controller_flag[RIGHT] = False
        super(QuickBot, self).set_pwm_right(pwm_right)


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

    def set_enc_vel(self, env_vel):
        """ Setter for encoder velocity values """
        for side in range(0, 2):
            self.enc_vel_set_point[side] = env_vel[side]
        self.enc_vel_controller_flag = 2*[True]

    def get_wheel_ang_vel(self):
        """ Getter for wheel angular velocity values """
        ang_vel = [0.0, 0.0]
        enc_vel = self.get_enc_vel()
        for side in range(0, 2):
            ang_vel[side] = enc_vel[side] * (2 * PI) / self.ticksPerTurn
        return ang_vel

    def set_wheel_ang_vel(self, ang_vel):
        """ Setter for wheel angular velocity values """
        for side in range(0, 2):
            self.enc_vel_set_point[side] = ang_vel[side] * self.ticksPerTurn / (2 * PI)
        self.enc_vel_controller_flag = 2*[True]


def enc_vel_controller_thread_fcn(self, side):
    """ Thread function for controlling for encoder tick velocity """
    while self.run_flag:
        if self.enc_vel_controller_flag[side]:
            x = self.enc_vel[side]
            u = self.pwm[side]
            x_bar = self.enc_vel_set_point[side]

            u_plus = enc_vel_controller(x, u, x_bar)

            if side == LEFT:
                super(QuickBot, self).set_pwm_left(u_plus)
            else:
                super(QuickBot, self).set_pwm_right(u_plus)

        time.sleep(self.sample_time)

def enc_vel_controller(x, u, x_bar):
    """ Wheel angular velocity controller """
    controller_type = 'PID'

    if controller_type == 'PID':
        P = 0.05
        u_plus = P * (x_bar - x) + u

    return u_plus


def read_ir_thread_fcn(self):
    """ Thread function for reading IR sensor values """
    while self.run_flag:
        for i in range(0, self.n_ir):
            ADC_LOCK.acquire()
            self.ir_val[i] = ADC.read_raw(config.IR[i])
            time.sleep(ADCTIME)
            ADC_LOCK.release()


def read_enc_val_thread_fcn(self, side):
    """ Thread function for reading encoder values """
    while self.run_flag:
        parse_encoder_buffer(self, side)
        time.sleep(self.sample_time)


def parse_encoder_buffer(self, side):
    """ Parses encoder serial data """
    enc_pos_update_flag = False
    enc_vel_update_flag = False

    t = time.time() - self.t0
    ts = t - self.enc_time[side]

    z = np.array([[np.NaN]])

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
                    enc_pos_update_flag = True

            v_pattern = r'V([0-9A-F]{4})'
            v_regex = re.compile(v_pattern)
            v_result = v_regex.findall(self.encoderBuffer[side])
            if len(v_result) >= 1:
                vel = utils.convertHEXtoDEC(v_result[-1], 4)

                if not math.isnan(vel) and not enc_vel_update_flag:
                    if side == RIGHT:
                        vel = -1*vel
                    z = np.array([[vel]])
                    enc_vel_update_flag = True

    u = self.pwm[side]
    x = self.enc_state[side]
    P = self.enc_state_cov[side]

    A = np.array([[1.0,  ts],
                  [0.0, 0.0]])
    B = np.array([[6.0]])
    C = np.array([[0.0, 1.0]])
    W = np.array([1.0, 1.0])
    V = np.array([0.5])

    (x_p, P_p) = utils.kalman(x, u, P, A, B, C, W, V, z)

    self.enc_state[side] = x_p
    self.enc_state_cov[side] = P_p

    self.enc_time[side] = t
    self.enc_vel[side] = np.asscalar(x_p[[1]])

    return enc_pos_update_flag and enc_vel_update_flag
