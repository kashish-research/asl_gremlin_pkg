#!/usr/bin/env python

# the above shebang line tells OS it is a python file

import rospy # import rospy package

# import string msg type from package std_msg
from std_msgs.msg import String 

# initialize the node called "topic_publisher"
rospy.init_node('comm_check_publish') 

# Advertise it as a publisher, publishing to topic damn a message type String
pub = rospy.Publisher('comms',String)

# Publish at a rate of 2 Hz
rt = 5
rate = rospy.Rate(rt)

count = 0
print("Publisher is running at %s Hz"%(rt))
# loop --> is_shutdown() returns true if node is ready 
# to be shutdown false otherwise
while not rospy.is_shutdown():
    pub.publish('Checking comms rover 1 do you read? at time step %d'%(count)) # publish the value of count
    count += 1
    rate.sleep() # makes sure the body of the while loop runs at 2 Hz
