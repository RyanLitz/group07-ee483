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

class FollowMM:
    def __init__(self):
        self.bridge = CvBridge()
        blobImage = Subscriber("~debug/detection_image/compressed",CompressedImage)
        detectFlag = Subscriber("~detection", BoolStamped)

        ats = ApproximateTimeSynchronizer([blobImage,detectFlag], queue_size=5, slop=0.1)
        ats.registerCallback(self.detect_MM)

        

    def detect_MM (self):
        print("f{detectFlag}")
        
    

if __name__=="__main__":
    rospy.init_node("follow_mm", anonymous=True)
    FollowMM()
    rospy.spin()