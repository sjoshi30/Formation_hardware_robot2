#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import rospy
import yaml
import numpy as np
import sys

from apriltag import apriltag
from DiffDriveController import DiffDriveController

class outerloop_apriltag(self):
    
    def __init__(self)

        # Handles apriltag measurements and transformations
        self.apriltag = apriltag(t_cam_to_body)

        #self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

    def process_measurements(self):

        meas = self.apriltag.get_measurements() 
        self.apriltag.command_velocity(0, 0)

        return 

def main(args):
    
    rospy.init_node('outerloop_apriltag')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)

    # Intialize the RobotControl object
    outerloop = outerloop_apriltag(max_vel, max_omega, t_cam_to_body)

    # Call process_measurements at 60Hz
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        outerloop.process_measurements()
        r.sleep()
    # Done, stop robot
    outerloop.outerloop_apriltag.command_velocity(0,0)

if _name_ == "_main_":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass

 
        
    
