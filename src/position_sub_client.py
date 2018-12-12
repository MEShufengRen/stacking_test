#!/usr/bin/env python
from stacking_test.msg import Location
import rospy
from stacking_test.srv import IKService
from std_msgs.msg import String


def callback(position):
    position.x=list(position.x)
    position.y = list(position.y)
    rospy.wait_for_service('move_arm')
    move_arm = rospy.ServiceProxy('move_arm', IKService)
    fail_position = Location()
    num = 0.0
    for i in range(len(position.x)):
        res = move_arm(position.x[i], position.y[i], num)
        print(res.flag)
        if res.flag == 1.0:
            num = num + 1.0
            print(num)
        else:
            print(num)
            fail_position.x.append(position.x[i])
            fail_position.y.append(position.y[i])

    for i in range(len(fail_position.x)):
        res = move_arm(fail_position.x[i], fail_position.y[i], num)
        if res.flag == 1.0:
            num = num + 1.0
    # if flag=='T':
    #     FINISH.data = '1'
    # finish_pub.publish(FINISH)

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