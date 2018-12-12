#!/usr/bin/env python
import rospy
from std_msgs.msg import String

rospy.init_node('confirmPos_node', anonymous=True)
pub = rospy.Publisher('/Confirm_pos', String, queue_size=10)
rate = rospy.Rate(100)
s = String()
s.data = '1'
First = 0
while not rospy.is_shutdown():
    rospy.sleep(3)
    if(First == 0):
        pub.publish(s)
        First =1
