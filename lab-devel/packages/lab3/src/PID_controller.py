#!/usr/bin/env python3
import sys
import rospy
import cv2
import matplotlib
import numpy as np
import os

from duckietown_msgs.msg import LanePose,Twist2DStamped

class PidController:
    def __init__(self):
        veh_name = os.environ['VEHICLE_NAME']
        rospy.Subscriber(f"/{veh_name}/lane_filter_node/lane_pose", LanePose, self.pidCallBack, queue_size=1)
        self.pub_pid = rospy.Publisher(f"/{veh_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)

        self.prevTime = None
        self.prevError = None
        self.cumulative_error = 0.0
    
    def pidCallBack(self, msg):

        car_cmd = Twist2DStamped()
        car_cmd.v = rospy.get_param("/velocity")

        kp = rospy.get_param("/kp")
        ki = rospy.get_param("/ki")
        kd = rospy.get_param("/kd")

        now = rospy.Time.now()

        phi = -msg.phi

        phi_list = []

        if(len(phi_list) < 5):
            phi_list.append(phi)
        else:
            #keeping the current 5
            phi_list.append(phi)
            phi_list.pop(0)

            phi = (phi_list[0] + phi_list[1] + phi_list[2] + phi_list[3] + phi_list[4]) / 5.0

        if(phi >= -0.15 and phi <= 0.15):
            phi = 0
        
        if(self.prevTime == None or self.prevError == None):
            #set current time to previous time for next loop
            self.prevTime = now

            #set current error to previous error
            self.prevError = phi

            return

        dt = (now - self.prevTime).to_sec()

        if(dt <= 0):
            #set current time to previous time for next loop
            self.prevTime = now

            #set current error to previous error
            self.prevError = phi

            return

        #code for integration
        self.cumulative_error += dt * phi

        #code for derivative
        deriv = (phi - self.prevError) / dt

        #make control signal based on formula from handout
        control_signal = kp*phi + ki*self.cumulative_error + kd*deriv

        car_cmd.omega = control_signal

        self.pub_pid.publish(car_cmd)

        #set current time to previous time for next loop
        self.prevTime = now

        #set current error to previous error
        self.prevError = phi

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("pidcontroller", anonymous=True)
    PidController()
    rospy.spin()