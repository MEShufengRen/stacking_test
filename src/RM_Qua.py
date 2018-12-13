#!/usr/bin/env python

import rospy
import tf
import numpy as np
from stacking_test.msg import Location
from sensor_msgs.msg import Range

rangez = 0.0
table_flag = True
zdistance = 66
Z_table = -0.17


def GetTansform_Range():
    tfRange = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (transRange, rotRange) = tfRange.lookupTransform('/base', '/reference/left_hand_range', rospy.Time(0))
            global rangez
            if(len(transRange)!=0):
                rangez = transRange[2]
                print(rangez)
                return rangez
                break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

def GetTransform():
    tfL = tf.TransformListener()
    tfR = tf.TransformerROS(True, rospy.Duration(10.0))
    while not rospy.is_shutdown():
        try:
            (trans, rot) = tfL.lookupTransform('/base', '/right_hand_camera', rospy.Time(0))
            if(len(trans)!=0):
                T = tfR.fromTranslationRotation(trans, rot)
                return np.linalg.inv(T)
                break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def callback(data):
    # print(data)
    global table_flag
    print(table_flag)
    if not table_flag:
        global Z_table
        global pub
        uL = data.x
        vL = data.y
        ET = GetTransform()
        print("in caLL BACK")
        print(Z_table)
        lenData = len(uL)
        posL = Location()
        list_x = []
        list_y = []
        for i in range(lenData):
            u = uL[i]
            v = vL[i]
            IntrinsicMatrix = np.matrix([[406.267, 0.0, 311.640, 0.0], [0.0, 406.267, 235.140, 0], [0, 0, 1, 0]])
            TotalMatrix = np.dot(IntrinsicMatrix, ET)
            A = np.concatenate((-TotalMatrix[:, 0:2], np.matrix([[u, v, 1]]).T), 1)
            B = np.matrix(TotalMatrix[:, 2]) * Z_table + np.matrix(TotalMatrix[:, 3])
            result = np.dot(np.linalg.inv(A), B)
            list_x.append(result[0]+0.035)
            list_y.append(result[1]-0.02)
        posL.x = list_x
        posL.y = list_y
        pub.publish(posL)

def Range_callback(data):
    global table_flag
    global zdistance
    global Z_table
    zdistance = data.range
    if zdistance < 0.5 and table_flag:
        rangez = GetTansform_Range()
        Z_table = rangez - zdistance + 0.0508
        # print("Z_table")
        # print(Z_table)
        table_flag = False


def listener():
    global pub
    rospy.init_node('world_location_publisher', anonymous=True)
    pub = rospy.Publisher('/world_location', Location, queue_size=10)
    rospy.Subscriber("location", Location, callback)
    rospy.Subscriber('/robot/range/left_hand_range/state', Range, Range_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        print("Shutting Down")