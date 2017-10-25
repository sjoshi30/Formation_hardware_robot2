#!/usr/bin/env python

'''
launchpad_node.py - Receive sensor values from Launchpad board and publish as topics

Created September 2014

Copyright(c) 2014 Lentin Joseph

Some portion borrowed from  Rainer Hessmer blog
http://www.hessmer.org/blog/
'''

#Python client library for ROS
import rospy
import sys
import time
import math

#This module helps to receive values from serial port
from SerialDataGateway import SerialDataGateway
#Importing ROS data types
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64
#Importing ROS data type for IMU
from sensor_msgs.msg import Imu

#Class to handle serial data from Launchpad and converted to ROS topics
class Launchpad_Class(object):
	
	def __init__(self):
		print "Initializing Launchpad Class"

		#Sensor variables
		self._Counter = 0

		self._left_encoder_value = 0
		self._right_encoder_value = 0

		self._left_wheel_speed_ = 0
		self._right_wheel_speed_ = 0

		#Get serial port and baud rate of Tiva C Launchpad
		port = rospy.get_param("~port", "/dev/ttyACM0")
		baudRate = int(rospy.get_param("~baudRate", 9600))

		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
		#Initializing SerialDataGateway with port, baudrate and callback function to handle serial data
		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)
		rospy.loginfo("Started serial communication")
		
		#Publisher for left and right wheel encoder values
		self._Left_Encoder = rospy.Publisher('lwheel',Int64,queue_size = 10)		
		self._Right_Encoder = rospy.Publisher('rwheel',Int64,queue_size = 10)		

		#Publisher for entire serial data
		self._SerialPublisher = rospy.Publisher('serial', String,queue_size=10)
		
		self.frame_id = '/base_link'

		self.deltat = 0
		self.lastUpdate = 0
 		
		self.pi = 3.14159
		
                #Speed subscriber
		self._left_motor_speed = rospy.Subscriber('left_wheel_speed',Float32,self._Update_Left_Speed)
		self._right_motor_speed = rospy.Subscriber('right_wheel_speed',Float32,self._Update_Right_Speed)


	def _Update_Left_Speed(self, left_speed):
		self._left_wheel_speed_ = left_speed.data
		rospy.loginfo(left_speed.data)
		speed_message = 's %d %d\r' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))
		self._WriteSerial(speed_message)
				

	def _Update_Right_Speed(self, right_speed):
		self._right_wheel_speed_ = right_speed.data
		rospy.loginfo(right_speed.data)
		speed_message = 's %d %d\r' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))
		self._WriteSerial(speed_message)

        # Calculate orientation from accelerometer and gyrometer
	def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))
		if(len(line) > 0):

			lineParts = line.split('\t')
			try:
				if(lineParts[0] == 'e'):
					self._left_encoder_value = long(lineParts[1])
					self._right_encoder_value = long(lineParts[2])


					self._Left_Encoder.publish(self._left_encoder_value)
					self._Right_Encoder.publish(self._right_encoder_value)
				
			except:
				rospy.logwarn("Error in Sensor values")
				rospy.logwarn(lineParts)
				pass
			

	def _WriteSerial(self, message):
		self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
		self._SerialDataGateway.Write(message)

	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()

	def Stop(self):
		rospy.logdebug("Stopping")
		self._SerialDataGateway.Stop()
			
if __name__ =='__main__':
	rospy.init_node('launchpad_ros',anonymous=True)
	launchpad = Launchpad_Class()
	try:
		
		launchpad.Start()	
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Error in main function")
	launchpad.Reset_Launchpad()
	launchpad.Stop()


