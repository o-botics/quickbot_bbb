#!/usr/bin/python
'''
Created on Jan 8, 2014

@author: Rowland O'Flaherty
'''

import time
import Adafruit_BBIO.ADC as ADC

LEFT = 0
RIGHT = 1

encoderPin = ('P9_39', 'P9_33')
encoderVal = [0, 0]
ADC.setup()

def update():
    encoderVal[LEFT] = ADC.read(encoderPin[LEFT])

if __name__ == '__main__':
    cnt = 0
    while cnt < 1000:
        cnt = cnt + 1
        update()
        print str(encoderVal[LEFT])
        time.sleep(0.01)
    
    

              