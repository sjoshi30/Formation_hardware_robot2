#!/usr/bin/env python



'''
launchpad_node.py - Receive sensor values from arduino and publish as topics

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
import serial

#This module helps to receive values from serial port
from SerialDataGateway import SerialDataGateway
#Importing ROS data types
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64
#Importing ROS data type for IMU
from sensor_msgs.msg import Imu


#ser = serial.Serial('/dev/ttyACM0', 9600)
#ser.write('0,0 \n')
#end = '\n'
#comma = ','
#vd = 0.5 
#wd = 0





#Class to handle serial data from Launchpad and converted to ROS topics
class Arduino_Class(object):
	
	def __init__(self):
		
                #print "Initializing Launchpad Class"

		#Sensor variables
		self._Counter = 0

		self._left_encoder_value = 0
		self._right_encoder_value = 0

		self._left_wheel_speed_ = 0
		self._right_wheel_speed_ = 0

		self._LastUpdate_Microsec = 0
		self._Second_Since_Last_Update = 0

		self.robot_heading = 0

		#Get serial port and baud rate of Tiva C Launchpad
		port = rospy.get_param("~port", "/dev/ttyACM0")
		baudRate = int(rospy.get_param("~baudRate", 115200))

		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))

		#Initializing SerialDataGateway with port, baudrate and callback function to handle serial data
		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)
		rospy.loginfo("Started serial communication")

                self.left_wheel_vel = 0
                self.right_wheel_vel = 0
                self.x_odomdata = 0 
                self.y_odomdata = 0
                self.theta_odomdata = 0
		
		#Publisher for left and right wheel encoder values
		#self._Left_Encoder = rospy.Publisher('lwheel',Int64,queue_size = 10)		
		#self._Right_Encoder = rospy.Publisher('rwheel',Int64,queue_size = 10)
                self.left_wheel_vel_pub = rospy.Publisher('lwheel_vel',Int64, queue_size = 10)
                self.right_wheel_vel_pub = rospy.Publisher('rwheel_vel',Int64, queue_size = 10)
                self.x_data_pub = rospy.Publisher('x_data',Int64, queue_size = 10)
                self.y_data_pub = rospy.Publisher('y_data',Int64, queue_size = 10)
                self.theta_data_pub = rospy.Publisher('theta_data',Int64, queue_size = 10)
		
	
		#Publisher for entire serial data
		self._SerialPublisher = rospy.Publisher('serial', String,queue_size=10)
		
                #Speed subscriber
		#self._left_motor_speed = rospy.Subscriber('left_wheel_speed',Float32,self._Update_Left_Speed)
		#self._right_motor_speed = rospy.Subscriber('right_wheel_speed',Float32,self._Update_Right_Speed)             

                self.pi = 3.14159

        def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))
		if(len(line) > 0):

			lineParts = line.split('\t')
			try:
				if(lineParts[0] == 'e'):
                                        self.left_wheel_vel = long(lineParts[1])
                                        self.right_wheel_vel = long(lineParts[2])
                                        self.x_odomdata = long(lineParts[3])
                                        self.y_odomdata = long(lineParts[4])
                                        self.theta_odomdata = long(lineParts[5])

                                        self.left_wheel_vel_pub.publish(self.left_wheel_vel) 
                                     	self.right_wheel_vel_pub.publish(self.right_wheel_vel) 
                                        self.x_data_pub.publish(self.x_odomdata)
                                        self.y_data_pub.publish(self.y_odomdata)
                                        self.theta_data_pub.publish(self.theta_odomdata) 	                                        	
			except:
				rospy.logwarn("Error in Sensor values")
				rospy.logwarn(lineParts)
				pass
			
	def _Update_Left_Speed(self, left_speed):
		self._left_wheel_speed_ = left_speed.data
		rospy.loginfo(left_speed.data)
		speed_message = ' %d , %d '  %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))      # 's %d %d\r' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))
		self._WriteSerial(speed_message)

	def _Update_Right_Speed(self, right_speed):
		self._right_wheel_speed_ = right_speed.data
		rospy.loginfo(right_speed.data)
		speed_message = ' %d , %d '  %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))      # 's %d %d\r' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))
		self._WriteSerial(speed_message)
        

	def _WriteSerial(self, message):
		self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
		self._SerialDataGateway.Write(message)

	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()
                #velocity_message = '%d , %d' %(100,0)
                #self._WriteSerial(velocity_message)

	def Stop(self):
		rospy.logdebug("Stopping")
		self._SerialDataGateway.Stop()
			

if __name__ =='__main__':

	rospy.init_node('arduino_node',anonymous=True)
	arduino = Arduino_Class()
	try:	
		arduino.Start()
                ###########################################
                #swr = str(vd)
                #swl = str(wd)
                #string = swr + comma + swl + end
                #ser.write(string)
                ###########################################	 
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Error in main function")

        ################################### 
        #ser.write('0,0 \n')
        #ser.close()
        ###################################

	arduino.Stop()



