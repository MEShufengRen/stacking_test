#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
import rospy
import numpy 
from std_msgs.msg import String
import roslib
import cv2
import imutils
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3 
from stacking_test.msg import Location
from CubePositionClass import CubePositionClass
from std_msgs.msg import String


"""
DEFINES
"""

(x,y) = (0,0)

OUTOFBOUNDS = -1

IMAGE_X = 640
IMAGE_Y= 480
QSIZE = 10
xqueue = []
yqueue = []
prev_x = 0
prev_y = 0

click_count = 0
marker = 0;
flag_arm = 1

def onMouse(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clickX = x
        clickY = y
        color = cv_image[clickY,clickX]
        rospy.loginfo(color)
        global boundary
        global blue, green, red
        blue = color[0]
        green = color[1]
        red = color[2]
        boundary = ([blue-30,green-30,red-30],[blue+30,green+30,red+30])
        global click_count
        click_count = click_count + 1
        rospy.loginfo("bound = %s",boundary)
        rospy.loginfo("current click time: %d", click_count)
        rospy.loginfo("marker: %f", marker)
        rospy.loginfo("------")

def camera_info_callback(camera_info):
    pass

def arm_finish_callback(finish):
    """
    FINISH = 0: Arm movement interrupted, publish previous location again -- FLAG_ARM = 2
    FINISH = 1: Arm movement finished, publish the next location -- FLAG_ARM = 1
    FLAG_ARM = 0: Waiting status
    """
    global flag_arm
    if finish.data == '1':
        flag_arm = 1
    elif finish.data == '0':
        flag_arm = 2
    # print("flag_arm")
    # print(flag_arm)
    # print("finish data")
    # print(finish.data)


def image_raw_callback(image):
    from cv_bridge import CvBridge, CvBridgeError
    bridge = CvBridge()
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    (rows,cols,channels) = cv_image.shape
    coords_for_object(cv_image)
    return

def coords_for_object(cv_image):
    global cubeposclass
    global flag_arm
    location_publisher = rospy.Publisher('/location', Location, queue_size=1)
    loc = Location()
    xlist = []
    ylist = []
    global prev_x, prev_y, marker
    # define the list of boundaries for BGR
    lower = numpy.array(boundary[0],dtype = "uint8")
    higher = numpy.array(boundary[1],dtype = "uint8")
    # lower = numpy.array(lower,dtype = "uint8")
    # higher = numpy.array(upper,dtype = "uint8")

    # find the colors within the specified boundaries and apply the mask
    mask = cv2.inRange(cv_image, lower, higher)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    # rospy.loginfo("%s",len(cnts))

    output = cv2.findNonZero(mask)
    output = cv2.bitwise_and(cv_image, cv_image, mask = mask)

    max_area = 0;
    max_x = 0;
    max_y = 0;
    max_angle = 0;
    # loop over the contours
    flag_box = False
    for c in cnts:
        # compute the center of the contour
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        area = cv2.contourArea(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = numpy.int0(box)
        if area > max_area:
            max_area = area
            max_x = cX
            max_y = cY
            max_angle = rect[-1]
            if click_count == 1:
                marker = rect[1][1]
            box1 = box
            flag_box = True
        cv2.drawContours(cv_image,[box],0,(255,0,0),2)
        xlist.append(cX)
        ylist.append(cY)
    
    # rospy.loginfo("data: %s",rect) 
    if flag_box:
        cv2.drawContours(output,[box1],0,(0,255,0),2)

    # rospy.loginfo("%s",ylist)
    # rospy.loginfo("===========")
    # loc.y.append(max_x/640.0*1.2-0.6)
    # loc.x.append(0.8-max_y/400.0*0.6)

    # if (numpy.abs(prev_x - max_x) > 15) & (numpy.abs(prev_y - max_y) > 15) & (flag_arm == 1):
    print("flag: ",flag_arm)
    if (flag_arm == 1):
        prev_x = max_x
        prev_y = max_y
        # loc.y.append(max_x / 640.0 * 1.2 - 0.6)
        # loc.x.append(0.8 - max_y / 400.0 * 0.6)
        loc.x.append(max_x )
        loc.y.append(max_y )
        loc.theta.append(max_angle)
        if(max_x> 0 or max_y>0):
            location_publisher.publish(loc)
            flag_arm = 0

    if flag_arm == 2:
        loc.x.append(prev_x )
        loc.y.append(prev_y )
        loc.theta.append(max_angle)
        location_publisher.publish(loc)
        flag_arm = 0

    print("x: ", max_x)
    print("y: ", max_y)

    # show images
    cv2.imshow("cv_images", numpy.hstack([cv_image, output]))
    cv2.setMouseCallback("cv_images",onMouse,0)
    #cv2.waitKey(3)
    return

def filter(x, y):
    global xqueue 
    global yqueue 

    if(len(xqueue)+1 > QSIZE):
        del xqueue[0]
    xqueue.append(x)
    avg = numpy.sum(yqueue) / QSIZE
    if(x < 0.8*avg  or x > 0.8*avg):
        x = OUTOFBOUNDS

    if(len(yqueue)+1 > QSIZE):
        del yqueue[0]
    xqueue.append(y)
    avg = numpy.sum(xqueue) / QSIZE
    if(y < 0.8*avg  or y > 0.8*avg):
        y = OUTOFBOUNDS
    return x, y
    
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
    pub = rospy.Publisher('batter', Vector3, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        global x, y
        v = Vector3(x, y, 0)
        pub.publish(v)
        cv2.waitKey(3)
        rate.sleep()
    
def listener():
    # rospy.Subscriber('/usb_cam/camera_info', CameraInfo, camera_info_callback)
    # rospy.Subscriber('/usb_cam/image_raw', Image, image_raw_callback)
    # rospy.Subscriber('/usb_cam/image_rect_color', Image, image_raw_callback)
    global  cubeposclass
    #cubeposclass = CubePositionClass()
    rospy.Subscriber('/cameras/right_hand_camera/camera_info', CameraInfo, camera_info_callback)
    rospy.Subscriber('/cameras/right_hand_camera/image', Image, image_raw_callback)
    rospy.Subscriber('/pick_finish', String, arm_finish_callback)

def main():
    global clickX
    global clickY
    clickX = 0
    clickY = 0
    global boundary
    # boundary = [([80, 0, 0], [255, 240, 55]),([28, 31, 133], [88, 91, 193]),([102, 163, 89], [162, 223, 149])]
    boundary = ([80, 0, 0], [255, 240, 55])
    rospy.init_node('image_process_node', anonymous=True)
    listener()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

if __name__ == '__main__':
  main()
