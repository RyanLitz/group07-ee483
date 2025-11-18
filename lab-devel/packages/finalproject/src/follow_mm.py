#!/usr/bin/env python3

import sys
import rospy
import cv2
import os
import numpy as np

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from duckietown_msgs.msg import BoolStamped
from sensor_msgs.msg import Range


class FollowMM:
    def __init__(self):
        self.bridge = CvBridge()
        blobImage = Subscriber("~debug/detection_image/compressed",CompressedImage)
        detectFlag = Subscriber("~detection", BoolStamped)
        

        ats = ApproximateTimeSynchronizer([blobImage,detectFlag], queue_size=5, slop=0.1)
        ats.registerCallback(self.detect_MM)

        rospy.Subscriber("/ee483mm07/front_center_tof_driver_node/range", Range, self.distanceBetween)

        self.detected = False

        
    #this function returns basically if the duckiebot detects another bot
    def detect_MM(self, image_msg, detect_msg):
        self.detected = detect_msg.data
        rospy.loginfo(f"[follow_mm] detection flag: {self.detected}")

    def distanceBetween(self, msg):
        dist = msg.range

        if self.detected:
            rospy.loginfo(f"[follow_mm] Range = {dist:.3f}")

            if dist <= 0.10:  
                rospy.logwarn("[follow_mm] Too close → stopping.")
                rospy.set_param("/velocity", 0.0)   
        else:
            rospy.loginfo("[follow_mm] No robot detected.")
        

        
    

if __name__=="__main__":
    rospy.init_node("follow_mm", anonymous=True)
    FollowMM()
    rospy.spin()