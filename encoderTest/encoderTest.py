#!/usr/bin/python
'''
Created on Jan 8, 2014

@author: Rowland O'Flaherty
'''

import Adafruit_BBIO.ADC as ADC

LEFT = 0
RIGHT = 1

encoderPin = ('P9_39', 'P9_33')
encoderVal = [0, 0]
ADC.setup()

if __name__ == '__main__':
    print "Hello World From BBB"