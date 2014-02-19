#!/usr/bin/python
import Adafruit_BBIO.UART as UART
import serial
 
UART.setup("UART4")
 
ser = serial.Serial(port = "/dev/ttyO4", baudrate=9600)
ser.close()
ser.open()
if ser.isOpen():
    print "Serial is open!"
    ser.write("Hello from BBB!\n")
ser.close()
