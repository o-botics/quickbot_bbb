# ultrabot
#
#                     _'_'_ UXfm
#                 ++++++++++++++
#             _/ +              + \_
#       UXfl _/ +                + \_ UXfr
#            / +     LMP   RMP    + \
#              +    __|     |__   +
#              +   |          |   +
#              +   |          |   +
#           _|++++ |  _    _  | ++++|_
#      UXbl _|++++_| T T  T T |_++++|_ UXbr
#            |++++   | |  | |   ++++|
#             ++++    Ol   Or   ++++
#              +                  +
#               ++++++++++++++++++
#
# ultrasonic
UTbl = "P8_12"
UEbl = "P8_11"

UTfl = "P9_21"
UEfl = "P9_22"

UTfm = "P9_23"
UEfm = "P9_24"

UTfr = "P9_25"
UEfr = "P9_26"

UTbr = "P9_27"
UEbr = "P9_30"

ULTRAS = ((UTbl, UEbl), (UTfl, UEfl), (UTfm, UEfm), (UTfr, UEfr), (UTbr, UEbr))

# encoder aka odometry
Ol = "P9_41"
Or = "P9_42"

# motors
INl1 = "P9_11"
INl2 = "P9_12"
PWMl = "P9_14"

INr1 = "P9_13"
INr2 = "P9_15"
PWMr = "P9_16"

RMP = (INr1, INr2, PWMr)
LMP = (INl1, INl2, PWMl)

# led
LED = "USR1"
