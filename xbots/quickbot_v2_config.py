# quickbot
#
#                     _'_'_ IRfm
#                 ++++++++++++++
#             _/ +              + \_
#       IRfl _/ +                + \_ IRfr
#            / +     LMP   RMP    + \
#              +    __|     |__   +
#              +   |          |   +
#              +   |          |   +
#           _|++++ |  _    _  | ++++|_
#      IRbl _|++++_| T T  T T |_++++|_ IRbr
#            |++++   | |  | |   ++++|
#             ++++    Ol   Or   ++++
#              +                  +
#               ++++++++++++++++++
#

class encoderSerial:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate

class motorPin:
    def __init__(self, dir1, dir2, pwm):
        self.dir1 = dir1
        self.dir2 = dir2
        self.pwm = pwm

# IR sensor Pins
irBL = "P9_40"  # IR back left pin
irFL = "P9_38"  # IR front left pin
irFM = "P9_36"  # IR front middle pin
irFR = "P9_35"  # IR front right pin
irBR = "P9_33"  # IR back right pin
irPins = (irBL, irFL, irFM, irFR, irBR)  # IR pin set

# Encoder (aka odometry) serials
    # TTYO1: Rx=P9_26  Tx=P9_24
    # TTYO2: Rx=P9_22  Tx=P9_21
    # TTYO4: Rx=P9_11  Tx=P9_13
    # TTYO5: Rx=P9_38  Tx=P9_38
enL= encoderSerial('/dev/ttyO1', 38400)  # Encoder left serial
enR = encoderSerial('/dev/ttyO2', 38400)  # Encoder right serial

# Motor pins
motorL = motorPin("P8_14", "P8_16", "P9_16")
motorR = motorPin("P8_12", "P8_10", "P9_14")

# INl1 = "P8_14"
# INl2 = "P8_16"
# PWMl = "P9_16"

# INr1 = "P8_12"
# INr2 = "P8_10"
# PWMr = "P9_14"

# RMP = (INr1, INr2, PWMr)
# LMP = (INl1, INl2, PWMl)

# LED pin
ledPin = "USR1"
