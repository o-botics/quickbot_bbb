"""
@brief Encoder class for SparkFun Magician Chassis encoder

@author Rowland O'Flaherty 
@date 08/27/2013
"""

import time
import Adafruit_BBIO.ADC as ADC

class Encoder:
    """The Encoder Class"""
    
    # === Class Properties ===
    # Parameters
    sampleTime = 0.001
    threshold = 0.68
    maxCnt = 3000
    tickPerRev = 16
    
    # ADC Pins
    pin = 'P9_39'
    
    # State
    t0 = -1
    t = -1
    tPrev = -1
    val = -1
    cog = -1
    tick = 0
    dir = 1
    pos = 0
    vel = 0
    
    # === Class Methods ===
    # Constructor
    def __init__(self,pin):
        
        # Initialize ADC
        ADC.setup()
        
        # Set pin
        self.pin = pin

    # Methods
    def run(self):
        cnt = 0
        self.t0 = time.time()
        
        while cnt < self.maxCnt:
            self.sample()
            if self.tick == 1:
                print 'Time: ' + str(self.t) + \
                  '\tPos: ' + str(self.pos) + \
                  '\tVel: ' + str(self.vel)
                
            time.sleep(self.sampleTime)
    
    def sample(self):
        t = time.time() - self.t0
        self.val = ADC.read_raw(self.pin)
        print "Pin " + self.pin + ": " + str(self.val)
                
        cogPrev = self.cog
        if self.val >= self.threshold:
            self.cog = 1
        else:
            self.cog = 0
         
        if cogPrev == 0 and self.cog == 1:
            # Tick
            self.tick = 1
            self.pos = self.pos + self.dir
            if self.tPrev != -1:
                self.vel = self.dir * (t - self.t)**(-1.0) / (self.tickPerRev)
            self.tPrev = self.t
            self.t = t
        else:
            # No Tick
            self.tick = 0
        
        