#!/usr/bin/env python

import rospy
import tf
import numpy as np
from random import gauss
import functools
from stacking_test.msg import Location


ERRRED = '\033[91m'
ENDC = '\033[0m'

def matmult(*x):
  return functools.reduce(np.dot, x)


def quat_to_axis_angle(Q):
    th = 2*np.arccos(Q[0])
    if np.abs(th) < 1e-12:
        w = np.zeros(3)
    else:
        w = Q[1:]/np.sin(th/2.0)
    return w, th


def hat(w):
    return np.array([
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0]])


def unhat(what):
    return np.array([-what[1,2], what[0,2], -what[0,1]])


def axis_angle_to_so3(w, th):
    return np.eye(3) + matmult(hat(w),np.sin(th)) + matmult(hat(w),hat(w))*(1-np.cos(th))


def so3_to_axis_angle(R):
    th = np.arccos((R.trace() - 1)/2)
    if (th <= 1e-12):
        th = 0.0
        w = 3*[1/np.sqrt(3)]
    else:
        w = 1/(2*np.sin(th))*np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
        if any(map(np.isinf, w)) or any(map(np.isnan, w)):
            th = 0.0
            w = 3*[1/np.sqrt(3)]
    return w, th


def quat_to_so3(Q):
    w, th = quat_to_axis_angle(Q)
    return axis_angle_to_so3(w, th)


def so3_to_quat(R):
    w,th = so3_to_axis_angle(R)
    return axis_angle_to_quat(w, th)


def quat_mult(Q,P):
    q0 = Q[0]
    p0 = P[0]
    q = Q[1:]
    p = P[1:]
    return np.hstack((q0*p0-np.dot(q,p), q0*p + p0*q + np.cross(q,p)))


def quat_conj(Q):
    return np.hstack((Q[0], -1*Q[1:]))


def quat_norm(Q):
    return np.linalg.norm(Q)


def quat_inv(Q):
    return quat_conj(Q)/quat_norm(Q)**2.0


def quat_transform(Q, p):
    pq = np.hstack((0,p))
    return quat_mult(quat_mult(Q,pq),quat_inv(Q))[1:]


def make_rand_unit_vector(dims):
    vec = [gauss(0, 1) for i in range(dims)]
    mag = sum(x**2 for x in vec) ** .5
    return [x/mag for x in vec]


def GetTransform():
    #rate = rospy.Rate(20)
    tfL = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (trans, rot) = tfL.lookupTransform('/base', '/right_hand_camera', rospy.Time(0))
            if(len(trans)!=0):
                break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    Rab = quat_to_so3(rot)
    T = np.concatenate((Rab, np.matrix(trans).reshape(3, 1)), axis=1)
    T = np.concatenate((T, np.matrix([[0, 0, 0, 1]])), axis=0)
    T = T.I
    return T


def callback(data):
    global pub
    uL = data.x
    vL = data.y
    Z = -0.1
    ET = GetTransform()
    lenData = len(uL)
    posL = Location()
    list_x = []
    list_y = []
    for i in range(lenData):
        u = uL[i]
        v = vL[i]
        # IntrinsicMatrix = np.array([[406.6258, 0, 307.8967, 0], [0, 406.8015, 234.9393, 0], [0, 0, 1, 0]])
        IntrinsicMatrix = np.array([[398.124176, 0, 303.086915, 0], [0, 403.328003, 243.216396, 0], [0, 0, 1, 0]])
        TotalMatrix = np.dot(IntrinsicMatrix, ET)
        A = np.concatenate((-TotalMatrix[:, 0:2], np.array([[u, v, 1]]).T), 1)
        B = np.array(TotalMatrix[:, 2]) * Z + np.array(TotalMatrix[:, 3])
        result = np.dot(A.I, B)
        list_x.append(result[0])
        list_y.append(result[1])
    posL.x = list_x
    posL.y = list_y
    pub.publish(posL)


def listener():
    global pub
    rospy.init_node('world_location_publisher', anonymous=True)
    pub = rospy.Publisher('/world_location', Location, queue_size=10)
    rospy.Subscriber("location", Location, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        print("Shutting Down")
