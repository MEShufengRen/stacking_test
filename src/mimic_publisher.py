#!/usr/bin/env python
from stacking_test.msg import Location
import rospy


def talker():
    pub = rospy.Publisher('location', Location, queue_size=10)
    rospy.init_node('mimic_publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cubic_position = Location()
        cubic_position.x = [0.4, 0.6, 0.5]
        cubic_position.y = [0.2, 0.2, -0.2]
        pub.publish(cubic_position)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass