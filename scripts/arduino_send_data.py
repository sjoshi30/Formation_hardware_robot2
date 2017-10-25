#!/usr/bin/env python

import numpy as np
import io
import serial
import matplotlib.pyplot as plt
import pylab as plab
import rospy
import sys
import time
import math

ser = serial.Serial('/dev/ttyACM0', 9600)
ser.write('0,0 \n')

end = '\n'
comma = ','

vd = 100 
wd = 0

if __name__ =='__main__':

	rospy.init_node('arduino_send_data',anonymous=True)
	try:	
		swr = str(vd)
                swl = str(wd)
                string = swr + comma + swl + end
                ser.write(string)
	 
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Error in main function")
        ser.write('0,0 \n')
        ser.close()

	arduino.Stop()


while(1):
    



    



