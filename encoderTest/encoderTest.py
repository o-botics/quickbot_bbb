#!/usr/bin/python
'''
Created on Jan 8, 2014

@author: Rowland O'Flaherty
'''
import sys
sys.path.append('..')
from Encoder import *

print "Running Encoder"
pin = 'P9_39'
e1 = Encoder(pin)
e1.run()