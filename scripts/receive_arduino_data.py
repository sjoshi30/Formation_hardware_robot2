# This python code is used to receive on the
# Raspberry Pi 2 data sent from Arduino board

import cv2  
from numpy import linalg as LA 
import numpy as np 
import io 
import picamera 
import serial 
import matplotlib.pyplot as plt 
import pylab as plab 
import time
import math

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)


data = 'data'

text = open(data, 'w')

while(1):
        
    a = ser.readline()
    a = a.rstrip()

    if len(a):
        text.write(a+'\n')
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

text.close()
