#!/usr/bin/env python

# Make sure it's using the python environment

import rospy
from std_msgs.msg import String


# Define a function to print the message to the screen
def callback(msg):
    print("I heard rover 2 message saying...")
    print msg.data

# Initialize the node called fucking_subscriber
rospy.init_node('comm_check_subscribe') 
 
# Create subscriber object to subscribe to damn topic
sub = rospy.Subscriber('comms', String, callback)

rospy.spin()
