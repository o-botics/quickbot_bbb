"""
@brief QuickBot class configuration module


                    _'_'_ IRfm
                ++++++++++++++
            _/ +              + \_
      IRfl _/ +                + \_ IRfr
           / +     LMP   RMP    + \
             +    __|     |__   +
             +   |          |   +
             +   |          |   +
          _|++++ |  _    _  | ++++|_
     IRbl _|++++_| T T  T T |_++++|_ IRbr
           |++++   | |  | |   ++++|
            ++++    Ol   Or   ++++
             +                  +
              ++++++++++++++++++

@author Rowland O'Flaherty (rowlandoflaherty.com)
@date 05/30/2014
@version: 2.0
@copyright: Copyright (C) 2014, see the LICENSE file
"""

class EncoderSerial(object):  #pylint: disable=R0903
    """
    Stores in encoder serial parameters.
    """
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate

class MotorPin(object):  #pylint: disable=R0903
    """
    Stores motor pin parameters.
    """
    def __init__(self, dir1, dir2, pwm):
        self.dir1 = dir1
        self.dir2 = dir2
        self.pwm = pwm

# IR sensor Pins
IR_BL = "P9_40"  # IR back left pin
IR_FL = "P9_38"  # IR front left pin
IR_FM = "P9_36"  # IR front middle pin
IR_FR = "P9_35"  # IR front right pin
IR_BR = "P9_33"  # IR back right pin
IR = (IR_BL, IR_FL, IR_FM, IR_FR, IR_BR)  # IR pin set

# Encoder (aka odometry) serials
    # TTYO1: Rx=P9_26  Tx=P9_24
    # TTYO2: Rx=P9_22  Tx=P9_21
    # TTYO4: Rx=P9_11  Tx=P9_13
    # TTYO5: Rx=P9_38  Tx=P9_38
EN_L = EncoderSerial('/dev/ttyO1', 38400)  # Encoder left serial
EN_R = EncoderSerial('/dev/ttyO2', 38400)  # Encoder right serial

# Motor pins
MOTOR_L = MotorPin("P8_14", "P8_16", "P9_16")
MOTOR_R = MotorPin("P8_12", "P8_10", "P9_14")

# LED pin
LED = "USR1"
