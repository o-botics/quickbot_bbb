#!/usr/bin/python
"""
@brief QuickBot class for Beaglebone Black

@author Rowland O'Flaherty 
@date 08/27/2013
"""
import os
import sys
import time
import math
import re
import socket
import threading
import numpy as np


import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC

# Tic Toc Variables
TICTOC_START = 0
TICTOC_COUNT = 0
TICTOC_MEAN = 0
TICTOC_MAX = -float('inf')
TICTOC_MIN = float('inf')

# Constants
LEFT = 0
RIGHT = 1
MIN = 0
MAX = 1

ADCTIME = 0.001

ENC_BUF_SIZE = 2**8
# ENC_BUF_SIZE = 2**13

ENC_IND = [0, 0]
ENC_TIME = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]
ENC_VAL = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]

ADC_LOCK = threading.Lock()

RUN_FLAG = True
RUN_FLAG_LOCK = threading.Lock()

class QuickBot():
    """The QuickBot Class"""

    # === Class Properties ===
    # Parameters
    sampleTime = 20.0 / 1000.0

    # Pins
    ledPin = 'USR1'

    # Motor Pins -- (LEFT, RIGHT)
    dir1Pin = ('P8_12', 'P8_14')
    dir2Pin = ('P8_10', 'P8_16')
    pwmPin = ('P9_14', 'P9_16')

    # ADC Pins
    irPin = ('P9_35', 'P9_33', 'P9_40', 'P9_36', 'P9_38')
    encoderPin = ('P9_39','P9_37')
    
    # Encoder count parameters
    winSize = 2**5 # Should be power of 2
    ticksPerTurn = 16 # Number of ticks on encoder disc
    minTickVelThreshold = 0.54 # Threshold on the slowest tick velocity
    minPWMThreshold = 45 # Threshold on the minimum magnitude of a PWM input value
    vel95PrctRiseTime = 1.0 # Time it takes tick velocity to get to 95% of steady state value

    # State -- (LEFT, RIGHT)
    pwm = [0, 0]
    
    irVal = [0.0, 0.0, 0.0, 0.0, 0.0]
    ithIR = 0
    
    encTime = [0.0, 0.0]
    encVal = [0.0, 0.0] 
    encVel = [0.0, 0.0]
    encVelVar = [0.1, 0.1]
    
    encSumN = [0, 0]
    encBufInd0 = [0, 0]
    encBufInd1 = [0, 0]
    encTimeWin = np.zeros((2,winSize))
    encValWin = np.zeros((2,winSize))
    encPWMWin = np.zeros((2,winSize))
    encTau = [0.0, 0.0]
    encCnt = 0;

    # Constraints
    pwmLimits = [-100, 100] # [min, max]

    # Variables
    ledFlag = True
    cmdBuffer = ''

    # UDP
    baseIP = '192.168.7.1'
    robotIP = '192.168.7.2'

    port = 5005
    robotSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robotSocket.setblocking(False)

    # === Class Methods ===
    # Constructor
    def __init__(self, baseIP, robotIP):
        
        # Initialize GPIO pins
        GPIO.setup(self.dir1Pin[LEFT], GPIO.OUT)
        GPIO.setup(self.dir2Pin[LEFT], GPIO.OUT)
        GPIO.setup(self.dir1Pin[RIGHT], GPIO.OUT)
        GPIO.setup(self.dir2Pin[RIGHT], GPIO.OUT)

        GPIO.setup(self.ledPin, GPIO.OUT)

        # Initialize PWM pins: PWM.start(channel, duty, freq=2000, polarity=0)
        PWM.start(self.pwmPin[LEFT], 0)
        PWM.start(self.pwmPin[RIGHT], 0)

        # Set motor speed to 0
        self.setPWM([0, 0])

        # Initialize ADC
        ADC.setup()
        self.encoderRead = encoderRead(self.encoderPin)

        # Set IP addresses
        self.baseIP = baseIP
        self.robotIP = robotIP
        self.robotSocket.bind((self.robotIP, self.port))

    # Getters and Setters
    def setPWM(self, pwm):
        # [leftSpeed, rightSpeed]: 0 is off, caps at min and max values

        self.pwm[LEFT] = min(max(pwm[LEFT], self.pwmLimits[MIN]), self.pwmLimits[MAX])
        self.pwm[RIGHT] = min(max(pwm[RIGHT], self.pwmLimits[MIN]), self.pwmLimits[MAX])
        print 'Setting motor PWMs to: left = ' + str(self.pwm[LEFT]) + ' and right = ' + str(self.pwm[RIGHT])

        # Left motor
        if self.pwm[LEFT] > 0:
            GPIO.output(self.dir1Pin[LEFT], GPIO.LOW)
            GPIO.output(self.dir2Pin[LEFT], GPIO.HIGH)
            PWM.set_duty_cycle(self.pwmPin[LEFT], abs(self.pwm[LEFT]))
        elif self.pwm[LEFT] < 0:
            GPIO.output(self.dir1Pin[LEFT], GPIO.HIGH)
            GPIO.output(self.dir2Pin[LEFT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[LEFT], abs(self.pwm[LEFT]))
        else:
            GPIO.output(self.dir1Pin[LEFT], GPIO.LOW)
            GPIO.output(self.dir2Pin[LEFT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[LEFT], 0)

        # Right motor
        if self.pwm[RIGHT] > 0:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.LOW)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.HIGH)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], abs(self.pwm[RIGHT]))
        elif self.pwm[RIGHT] < 0:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.HIGH)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], abs(self.pwm[RIGHT]))
        else:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.LOW)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], 0)

    # Methods
    def run(self):
        global RUN_FLAG
        self.encoderRead.start()
        
        while RUN_FLAG == True:
#             tic()
            self.update()
             
            # Flash BBB LED
            if self.ledFlag == True:
                self.ledFlag = False
                GPIO.output(self.ledPin, GPIO.HIGH)
            else:
                self.ledFlag = True
                GPIO.output(self.ledPin, GPIO.LOW)
            time.sleep(self.sampleTime)
#             toc("Run loop")
            
        self.cleanup()        
        return

    def cleanup(self):
        print "Clean up"
        self.setPWM([0, 0])
        self.robotSocket.close()
        GPIO.cleanup()
        PWM.cleanup()
#         tictocPrint()     
        # self.writeBufferToFile()

    def update(self):        
        self.readIRValues()        
        self.readEncoderValues()        
        self.parseCmdBuffer()


    def parseCmdBuffer(self):
        global RUN_FLAG
        try:
            line = self.robotSocket.recv(1024)
        except socket.error as msg:
            return
        
        self.cmdBuffer += line

        bufferPattern = r'\$[^\$\*]*?\*' # String contained within $ and * symbols with no $ or * symbols in it
        bufferRegex = re.compile(bufferPattern)
        bufferResult = bufferRegex.search(self.cmdBuffer)

        if bufferResult:
            msg = bufferResult.group()
            print msg
            self.cmdBuffer = ''

            msgPattern = r'\$(?P<CMD>[A-Z]{3,})(?P<SET>=?)(?P<QUERY>\??)(?(2)(?P<ARGS>.*)).*\*'
            msgRegex = re.compile(msgPattern)
            msgResult = msgRegex.search(msg)

            if msgResult.group('CMD') == 'CHECK':
                self.robotSocket.sendto('Hello from QuickBot\n',(self.baseIP, self.port))

            elif msgResult.group('CMD') == 'PWM':
                if msgResult.group('QUERY'):
                    self.robotSocket.sendto(str(self.pwm) + '\n',(self.baseIP, self.port))

                elif msgResult.group('SET') and msgResult.group('ARGS'):
                    args = msgResult.group('ARGS')
                    pwmArgPattern = r'(?P<LEFT>[-]?\d+),(?P<RIGHT>[-]?\d+)'
                    pwmRegex = re.compile(pwmArgPattern)
                    pwmResult = pwmRegex.match(args)
                    if pwmResult:
                        pwm = [int(pwmRegex.match(args).group('LEFT')), \
                        int(pwmRegex.match(args).group('RIGHT'))]
                        self.setPWM(pwm)

            elif msgResult.group('CMD') == 'IRVAL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.irVal)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))

            elif msgResult.group('CMD') == 'ENVAL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.encVal)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))

            elif msgResult.group('CMD') == 'ENVEL':
                if msgResult.group('QUERY'):
                    reply = '[' + ', '.join(map(str, self.encVel)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))
                    
            elif msgResult.group('CMD') == 'RESET':
                self.encVal[LEFT] = 0.0
                self.encVal[RIGHT] = 0.0
                print 'Encoder values reset to [' + ', '.join(map(str, self.encVel)) + ']'

            elif msgResult.group('CMD') == 'UPDATE':
                if msgResult.group('SET') and msgResult.group('ARGS'):
                    args = msgResult.group('ARGS')
                    pwmArgPattern = r'(?P<LEFT>[-]?\d+),(?P<RIGHT>[-]?\d+)'
                    pwmRegex = re.compile(pwmArgPattern)
                    pwmResult = pwmRegex.match(args)
                    if pwmResult:
                        pwm = [int(pwmRegex.match(args).group('LEFT')), \
                        int(pwmRegex.match(args).group('RIGHT'))]
                        self.setPWM(pwm)

                    reply = '[' + ', '.join(map(str, self.encVal)) + ', ' \
                      + ', '.join(map(str, self.encVel)) + ']'
                    print 'Sending: ' + reply
                    self.robotSocket.sendto(reply + '\n', (self.baseIP, self.port))

            elif msgResult.group('CMD') == 'END':
                print 'Quitting QuickBot run loop'
                RUN_FLAG_LOCK.acquire()
                RUN_FLAG = False
                RUN_FLAG_LOCK.release()

    def readIRValues(self):
        prevVal = self.irVal[self.ithIR]
        ADC_LOCK.acquire()
        self.irVal[self.ithIR] = ADC.read_raw(self.irPin[self.ithIR])
        time.sleep(ADCTIME)
        ADC_LOCK.release()
        
        if self.irVal[self.ithIR] >= 1100:
                self.irVal[self.ithIR] = prevVal
        
        self.ithIR = ((self.ithIR+1) % 5)
        
            
    def readEncoderValues(self):
        self.encCnt = self.encCnt + 1;
        # Fill window
        for side in range(0,2):
            self.encTime[side] = self.encTimeWin[side][-1]
            
            self.encBufInd0[side] = self.encBufInd1[side]
            self.encBufInd1[side] = ENC_IND[side]
            ind0 = self.encBufInd0[side] # starting index
            ind1 = self.encBufInd1[side] # ending index (this element is not included until the next update)
            
            if ind0 < ind1:
                N = ind1 - ind0 # number of elements
                self.encSumN[side] = self.encSumN[side] + N
                self.encTimeWin[side] = np.roll(self.encTimeWin[side], -N)
                self.encTimeWin[side, -N:] = ENC_TIME[side][ind0:ind1]
                self.encValWin[side] = np.roll(self.encValWin[side], -N)
                self.encValWin[side, -N:] = ENC_VAL[side][ind0:ind1]
                self.encPWMWin[side] = np.roll(self.encPWMWin[side], -N)
                self.encPWMWin[side, -N:] = [self.pwm[side]]*N
                
            elif ind0 > ind1:
                N = ENC_BUF_SIZE - ind0 + ind1 # number of elements
                self.encSumN[side] = self.encSumN[side] + N
                self.encTimeWin[side] = np.roll(self.encTimeWin[side], -N)
                self.encValWin[side] = np.roll(self.encValWin[side], -N)
                self.encPWMWin[side] = np.roll(self.encPWMWin[side], -N)
                self.encPWMWin[side, -N:] = [self.pwm[side]]*N
                if ind1 == 0:
                    self.encTimeWin[side, -N:] = ENC_TIME[side][ind0:]
                    self.encValWin[side, -N:] = ENC_VAL[side][ind0:]
                else:
                    self.encTimeWin[side, -N:-ind1] = ENC_TIME[side][ind0:]
                    self.encValWin[side, -N:-ind1] = ENC_VAL[side][ind0:]
                    self.encTimeWin[side, -ind1:] = ENC_TIME[side][0:ind1]
                    self.encValWin[side, -ind1:] = ENC_VAL[side][0:ind1]
                
            if ind0 != ind1:                
                tauNew = self.encTimeWin[side,-1] - self.encTimeWin[side,-N]
                self.encTau[side] = tauNew / self.encCnt + self.encTau[side] * (self.encCnt-1)/self.encCnt # Running average
                if self.encSumN[side] > self.winSize:
                    self.countEncoderTicks(side)
             
    def countEncoderTicks(self,side):
        # Set variables
        t = self.encTimeWin[side] # Time vector of data (not consistent sampling time)
        N = self.winSize # Number of samples
        T = np.linspace(t[0],t[-1],N)  # Time vector of new resampled data (consistent sampling time)
        Ts = np.mean(np.diff(T)) # Sampling time
        
        # Resample encoder values
        y = np.interp(T,t,self.encValWin[side]) # Encoder resampled data
        yBar = np.mean(y) # Average encoder values
        
        # FFT
        Fs = 1/Ts # Frequency sampling spacing
        Y = np.fft.fft(y-yBar,self.winSize) / N # Frequency spectrum of y
        f = Fs/2 * np.linspace(0,1,N/2+1) # Frequency vector
        
        YMag = 2*np.abs(Y[0:N/2+1]) # Single sided amplitude spectrum
        
        # Extract highest magnitude frequency component
        ind = np.argmax(YMag)
        mag = YMag[ind]
        freq = f[ind]
        
        freqRatio = mag / np.sum(YMag)  # Ratio of best frequency component energy to total energy in signal - Higher is better
        
        # Estimate tick velocity
        uStar = np.mean(self.encPWMWin[side]) # Input operating point
        
        # omega - Measured tick velocity
        if self.encVel[side] != 0:
            omega = freq / self.ticksPerTurn * np.sign(self.encVel[side])
        else:
            omega = freq / self.ticksPerTurn * np.sign(uStar)
            
        omegaStar = operatingPoint(uStar, self.minPWMThreshold) # Steady state tick velocity given current input                
        z = omega - omegaStar # Measurement tick velocity error from steady state value
        x = self.encVel[side] - omegaStar # Previous state value (state = error from steady state value)
        P = self.encVelVar[side] # Previous state variance
        A = -3 * 1 / self.vel95PrctRiseTime  # Continuous time state model matrix (xDot = A*x + B*u + w) (95% rise time)
        Phi = np.exp(A * self.encTau[side])
        H = 1; # Observation model matrix (z = H*x + v)
        W = 0.15 # Process noise covariance
        
        # Input is 0 and velocity is small set measurement to 0 with no noise
        # V - Measurement noise covariance
        if np.abs(uStar) < self.minPWMThreshold and np.abs(x) < self.minTickVelThreshold:
            z = 0
            V = 0
        else:
            # Measurement noise is bigger when spike in frequency spectrum is less pronounced
            V = max(0.1 - (freqRatio-.25)/8, 0.01);
            
        # Kalman filter
        (xPlus, PPlus) = kalman(x,P,Phi,H,W,V,z)
        
        # Estimate tick velocity
        self.encVel[side] = xPlus + omegaStar
        self.encVelVar[side] = PPlus
        
        # Count ticks        
        ticksNew = self.encVel[side] * self.ticksPerTurn * (t[-1] - self.encTime[side])
        self.encVal[side] = ticksNew + self.encVal[side]        
        
    
    def writeBufferToFile(self):
        matrix = map(list, zip(*[self.encTimeBuf[LEFT], self.encValBuf[LEFT], self.encPWMBuf[LEFT], self.encNNewBuf[LEFT], \
                                 self.encTimeBuf[RIGHT], self.encValBuf[RIGHT], self.encPWMBuf[RIGHT], self.encNNewBuf[RIGHT]]))
        s = [[str(e) for e in row] for row in matrix]
        lens = [len(max(col, key=len)) for col in zip(*s)]
        fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
        table = [fmt.format(*row) for row in s]
        f = open('output.txt','w')
        f.write('\n'.join(table))
        f.close()
        print "Wrote buffer to output.txt"
            

class encoderRead(threading.Thread):
    """The encoderRead Class"""
    
    # === Class Properties ===
    # Parameters
    
    # === Class Methods ===
    # Constructor
    def __init__(self,encPin=('P9_39', 'P9_37')):
        
        # Initialize thread
        threading.Thread.__init__(self)
        
        # Set properties
        self.encPin = encPin
        
    # Methods
    def run(self):
        global RUN_FLAG
        
        self.t0 = time.time()
        while RUN_FLAG:
            global ENC_IND
            global ENC_TIME
            global ENC_VAL
            
            for side in range(0,2):
                ENC_TIME[side][ENC_IND[side]] = time.time() - self.t0
                ADC_LOCK.acquire()
                ENC_VAL[side][ENC_IND[side]] = ADC.read_raw(self.encPin[side])
                time.sleep(ADCTIME)
                ADC_LOCK.release()
                ENC_IND[side] = (ENC_IND[side] + 1) % ENC_BUF_SIZE
                

def operatingPoint(uStar, uStarThreshold):
    """ This function returns the steady state tick velocity given some PWM input.
    
    uStar: PWM input.
    uStarThreshold: Threshold on the minimum magnitude of a PWM input value
    
    returns: omegaStar - steady state tick velocity
    """
    # Matlab code to find beta values
    # X = [40; 80; 100];
    # Y = [0.85; 2.144; 3.5];
    # H = [X ones(size(X))];
    # beta = H \ Y
    beta = [0.0425, -0.9504]
    
    if np.abs(uStar) <= uStarThreshold:
        omegaStar = 0.0
    elif uStar > 0:
        omegaStar = beta[0]*uStar + beta[1]
    else:
        omegaStar = -1.0*(beta[0]*np.abs(uStar) + beta[1])
        
    return omegaStar
    

def kalman(x, P, Phi, H, W, V, z):
    """This function returns an optimal expected value of the state and covariance 
    error matrix given an update and system parameters.
    
    x:   Estimate of staet at time t-1.
    P:   Estimate of error covariance matrix at time t-1.
    Phi: Discrete time state tranistion matrix at time t-1.
    H:   Observation model matrix at time t.
    W:   Process noise covariance at time t-1.
    V:   Measurement noise covariance at time t.
    z:   Measurement at time t.
    
    returns: (x,P) tuple
    x: Updated estimate of state at time t.
    P: Updated estimate of error covariance matrix at time t. 
       
    """
    x_p = Phi*x # Prediction of setimated state vector    
    P_p = Phi*P*Phi + W # Prediction of error covariance matrix
    S = H*P_p*H + V # Sum of error variances
    S_inv = 1/S # Inverse of sum of error variances
    K = P_p*H*S_inv # Kalman gain
    r = z - H*x_p # Prediction residual
    w = -K*r # Process error
    x = x_p - w # Update estimated state vector
    v = z - H*x # Measurement error
    if np.isnan(K*V):
        P = P_p
    else:
        P = (1 - K*H)*P_p*(1 - K*H) + K*V*K # Updated error covariance matrix

    return (x, P)

def tic():
    global TICTOC_START
    TICTOC_START = time.time()
    
def toc(tictocName='toc', printFlag=True):
    global TICTOC_START
    global TICTOC_COUNT
    global TICTOC_MEAN
    global TICTOC_MAX
    global TICTOC_MIN
       
    tictocTime = time.time() - TICTOC_START
    TICTOC_COUNT = TICTOC_COUNT + 1
    TICTOC_MEAN = tictocTime / TICTOC_COUNT + TICTOC_MEAN * (TICTOC_COUNT-1) / TICTOC_COUNT
    TICTOC_MAX = max(TICTOC_MAX,tictocTime)
    TICTOC_MIN = min(TICTOC_MIN,tictocTime)
    
    if printFlag:
        print tictocName + " time: " + str(tictocTime)
        
def tictocPrint():
    global TICTOC_COUNT
    global TICTOC_MEAN
    global TICTOC_MAX
    global TICTOC_MIN
    
    print "Tic Toc Stats:"
    print "Count = " + str(TICTOC_COUNT)
    print "Mean = " + str(TICTOC_MEAN)
    print "Max = " + str(TICTOC_MAX)
    print "Min = " + str(TICTOC_MIN)

            
