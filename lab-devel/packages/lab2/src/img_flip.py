#!/usr/bin/env python3
import sys
import rospy
import cv2
import os
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class ImageFilter:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("ee483mm07/camera_node/image/compressed", CompressedImage, self.filter, queue_size=1, buff_size=10000000) #BUFF SIZE 10MB
        self.pub = rospy.Publisher("mirrored", Image, queue_size=1)
        self.filterPub = rospy.Publisher("image_white", Image, queue_size=1)

        
    def flipper_cb(self, msg):
        # convert to a ROS image using the bridge
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        # flip along the horizontal axis using an OpenCV function
        cv_flipped = cv2.flip(cv_img, 1)
        # convert new image to ROS to send
        ros_flipped = self.bridge.cv2_to_imgmsg(cv_flipped, "bgr8")
        # publish flipped image
        self.pub.publish(ros_flipped)
        

    def filter(self, msg):

        #Pulling image from ROS msg to cv format
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        #Convert image from BGR to HS1
        height, _, _ = cv_img.shape

        # Crop to the bottom half
        cropped_img = cv_img[height//3:, :]  # rows from half-height to end, all columns
        #Converting image from cv format to ROS msg
        hsv_image = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        #Finds all pixels within the given range. In this scenario its white
        mask1 = cv2.inRange(hsv_image, (0, 0, 180), (180, 65, 255))
        mask2 = cv2.inRange(hsv_image, (14, 0, 180), (35, 255, 255))

        bigmask = mask1 + mask2
        #Converting image from cv format to ROS msg using mono8 since it is in grayscale
        ros_white = self.bridge.cv2_to_imgmsg(bigmask, "mono8")
        #publishing cropped image
        self.filterPub.publish(ros_white)

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("image_filter", anonymous=True)
    img_filter = ImageFilter()
    rospy.spin()