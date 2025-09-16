#!/usr/bin/env python3
import rospy
import os
import time
from duckietown_msgs.msg import WheelsCmdStamped # Import the message for the wheel comm
class Driver():#CHANGE CLASSNAME to the name of your class
    def __init__(self):
        self.veh_name = os.environ['VEHICLE_NAME']
        self.pub = rospy.Publisher('/ee483mm07/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10) #our publisher
        # USING PARAMETER TO GET THE NAME OF THE VEHICLE
        # THIS WILL BE USEFUL TO SPECIFY THE NAME OF THE TOPIC
        # INITIALIZE YOUR VARIABLES HERE (SUBSCRIBERS OR PUBLISHERS)
    def turn90(self):
        #commands to turn 90 degrees
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()

        cmd_to_publish.vel_right = 0.2
        cmd_to_publish.vel_left = -0.2
        self.pub.publish(cmd_to_publish)

        time.sleep(1.15)

        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0
        self.pub.publish(cmd_to_publish)

    def moveForward(self):
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()

        cmd_to_publish.vel_right = 0.5
        cmd_to_publish.vel_left = 0.5

        self.pub.publish(cmd_to_publish)
        
        #print("Driving the MM " + self.veh_name + " around the block") # Just for testin
        
        time.sleep(3.5)
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)


    def square(self): # CHANGE TO THE NAME OF YOUR FUNCTION
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()

        time.sleep(1)

        for i in range(4):
            self.pub.publish(cmd_to_publish)
            time.sleep(0.5)
            drive.moveForward()
            self.pub.publish(cmd_to_publish)
            time.sleep(0.5)
            drive.turn90()
            self.pub.publish(cmd_to_publish)
        
        self.pub.publish(cmd_to_publish)
        rospy.signal_shutdown("complete")
        self.pub.publish(cmd_to_publish)

        #WRITE THE CODE TO MAKE THE MM GO AROUND THE BLOCK
if __name__ == "__main__": ## The main function which will be called when your python sc
# Initialize the node
    try:
        rospy.init_node('driving')
        drive = Driver() # Create obj of the Driver class
        rospy.sleep(3) # Delay to wait enough time for the code to run
        # Keep the line above - you might be able to reduce the delay a bit,
        while not rospy.is_shutdown(): # Run ros forever - you can change
            # this as well instead of running forever
            drive.square() # calling your node function
    except rospy.ROSInterruptException:
        pass