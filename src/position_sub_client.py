#!/usr/bin/env python
from stacking_test.msg import Location
import rospy
from stacking_test.srv import IKService


def callback(position):
    print(position)
    position.x=list(position.x)
    #print(position.x)
    position.y = list(position.y)
    #print(position.y)
    rospy.wait_for_service('move_arm')
    move_arm = rospy.ServiceProxy('move_arm', IKService)
    for i in range(len(position.x)):
        print(position.x[i])
        print(position.y[i])
        flag = move_arm(position.x[i], position.y[i])
        # if flag == 'T':
        #     position.x.remove(position.x[i])
        #     position.y.remove(position.y[i])
        # else:
        #     position.x.remove(position.x[i])
        #     position.y.remove(position.y[i])
        #     position.x.insert(-1, position.x[i])
        #     position.y.insert(-1, position.y[i])


def main():
    rospy.init_node('position_pub_client', anonymous=True)
    rate = rospy.Rate(100)
    rospy.Subscriber('/location', Location, callback)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting Down")