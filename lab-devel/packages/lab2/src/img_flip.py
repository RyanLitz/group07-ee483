#!/usr/bin/env python3
import sys
import rospy
import cv2
import os
import numpy as np

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class ImageFilter:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("ee483mm07/camera_node/image/compressed", CompressedImage, self.filter, queue_size=1, buff_size=10000000) #BUFF SIZE 10MB
        self.pub = rospy.Publisher("mirrored", Image, queue_size=1)
        self.filterPub = rospy.Publisher("image_white", Image, queue_size=1)
        self.edgePub = rospy.Publisher("image_edges", Image, queue_size=1)
        self.linesPub = rospy.Publisher("image_lines", Image, queue_size=1)

    def filter(self, msg):

        #Pulling image from ROS msg to cv format
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        cv_flipped = cv2.flip(cv_img, 1)
        # convert new image to ROS to send
        ros_flipped = self.bridge.cv2_to_imgmsg(cv_flipped, "bgr8")
        # publish flipped image
        
        #Convert image from BGR to HS1
        
        height, _, _ = cv_img.shape

        # Crop to the bottom half
        cropped_img = cv_img[height//2:, :]  # rows from half-height to end, all columns
        #Converting image from cv format to ROS msg
        hsv_image = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        #Finds all pixels within the given range. In this scenario its white
        mask1 = cv2.inRange(hsv_image, (97,0,140), (180,65,255))
        mask2 = cv2.inRange(hsv_image, (17,0,139), (41,255,255))

        bigmask = mask1 + mask2
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        image_dialate = cv2.dilate(bigmask, kernel)
        #Converting image from cv format to ROS msg using mono8 since it is in grayscale
        ros_white = self.bridge.cv2_to_imgmsg(image_dialate, "mono8")
        #publishing cropped image

        gray = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        masked_gray = cv2.bitwise_and(gray, bigmask)

        blurred_mask_gray = cv2.GaussianBlur(masked_gray, (5,5),0)

        #appy canny filter
        edges = cv2.Canny(blurred_mask_gray, 100, 200)

        #edges = cv2.dilate(edges, np.ones((3,3), np.uint8), iterations=1)

        ros_edges = self.bridge.cv2_to_imgmsg(edges, "mono8")
        

        #apply hough transform
        lines = cv2.HoughLinesP(edges,1,np.pi/180, 20, minLineLength=30, maxLineGap=5)

        output = np.copy(cropped_img)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
                

        # Publish combined image with lines
        ros_lines = self.bridge.cv2_to_imgmsg(output, "bgr8")

        self.pub.publish(ros_flipped)
        self.filterPub.publish(ros_white)
        self.edgePub.publish(ros_edges)
        self.linesPub.publish(ros_lines)

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("image_filter", anonymous=True)
    img_filter = ImageFilter()
    rospy.spin()