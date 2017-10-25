#!/usr/bin/env python

import numpy as np
import rospy
import roslib

from std_msgs.msg import (
    Header,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Quaternion,
)
from tf.transformations import *

from apriltags_ros.msg import (
    AprilTagDetectionArray,
    AprilTagDetection,
)

from sensor_msgs.msg import Imu

from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    Pose,
    Twist,
)

import cv2
import yaml
import sys

# Extra utility functions
from utility import *

class apriltag(object):
   
    def __init__(self):
        self.no_detection = True ;
        self.T = None ;
        self.R = None ;
        self.R_cam2bot = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
        self.T_cam2bot = t_cam_to_body
        self.R_tag2bot = np.array([[0,-1,0,0],[0,0,1,0],[-1,0,0,0],[0,0,0,1]])
 
        # Publisher
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

        # Subscriber
        rospy.Subscriber("/camera/tag_detections",AprilTagDetectionArray,self.tag_pose_callback)

    def tag_pose_callack(self,posearray):
        
        if (len(posearray.detections)==0) :
            return 
        (self.T,self.R) =  get_t_R(posearray.detections[0].pose.pose)  
        self.angle = -np.arctan2(-self.R[2,0],np.sqrt(self.R[2,0]**2+self.R[2,2]**2))
        if math.isnan(self.angle):
            return
        self.R = np.dot(np.dot(self.R_cam2bot, self.R),self.R_tag2bot)
        self.T = np.dot(self.R_cam2bot, self.T)+self.T_cam2bot
        self.marker_num = posearray.detections[0].id
        self.no_detection = False

    def get_measurements(self):
        """
        Returns information about the last tag seen if any. Returns (x,y,theta) as a
        3x1 numpy array. Returns None if no new tag is seen.
        """
        if self.no_detection:
            return None
        self.no_detection = True
        dx = self.T[0,0]
        dy = self.T[1,0]
        return [[dx,dy,self.angle,self.marker_num]]

    def command_velocity(self,vx,wz):
        """
        Commands the robot to move with linear velocity vx and angular
        velocity wz
        """
        twist=Twist()
        twist.linear.x = vx
        twist.angular.z = wz
        self.pub.publish(twist)
        
       
        



    
 

    
        
    

    


