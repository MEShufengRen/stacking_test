#!/usr/bin/env python
from stacking_test.msg import Location
import rospy
from stacking_test.srv import IKService
from std_msgs.msg import String


def callback(position):
    global  finish_pub
    FINISH = String()
    FINISH.data = '1'
    flag = 'F'
    # print(position)
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

    if flag=='T':
        FINISH.data = '1'

    finish_pub.publish(FINISH)



def main():
    global  finish_pub
    rospy.init_node('position_pub_client', anonymous=True)
    rate = rospy.Rate(100)
    rospy.Subscriber('/world_location', Location, callback)
    finish_pub = rospy.Publisher('/pick_finish', String, queue_size=1)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting Down")