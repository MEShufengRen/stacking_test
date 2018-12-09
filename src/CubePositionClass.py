#!/usr/bin/env python
import rospy
import cv2
#import image_geometry
from image_geometry import PinholeCameraModel
from image_geometry import StereoCameraModel
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import tf
from geometry_msgs.msg import PointStamped
from stacking_test.msg import Location
from geometry_msgs.msg import PoseStamped
from right_moveIt import MoveItPyPlanner_right


class CubePositionClass(object):
    def __init__(self):
        #rospy.init_node('/testClass_node', anonymous=True)
        self._myPlanner_right = MoveItPyPlanner_right()
        self._myPlanner_right.move_robotArm(0.6, -0.1, 0.25)
        self.cameraZ = self._myPlanner_right.group.get_current_pose().pose.position.z+0.17
        self.pm = PinholeCameraModel()
        self.PMINIT = False
        self.tf_listener_ = tf.TransformListener()
        self.TFREADY = False
        self.testTf()
        rospy.Subscriber('/cameras/right_hand_camera/camera_info', CameraInfo, self.cameraInfo_callback)
        pass

    def testTf(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener_.lookupTransform('/base', '/right_hand_camera', rospy.Time(0))
                if (len(trans) != 0):
                    # self.cameraZ = trans[2]+0.17
                    # print('in tf')
                    # print(trans)
                    # print(rot)
                    self.TFREADY = True
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        pass

    def compute_uVector(self,uvList):
        if(self.PMINIT):
            uList = uvList.x
            vList = uvList.y
            length = len(uvList.x)
            uVectorList = []
            for i in range(length):
                uvTemp=(uList[i],vList[i])
                uVectorList.append(self.pm.projectPixelTo3dRay(uvTemp))
            return uVectorList
        else:
            return []

    def compute_cVector(self,uVectorList):
        lenth = len(uVectorList)
        self.cameraZ = self._myPlanner_right.group.get_current_pose().pose.position.z + 0.17
        cVectorList = []
        for i in range(lenth):
            scale =  self.cameraZ /uVectorList[i][2]
            tempX = uVectorList[i][0]*scale
            tempY = uVectorList[i][1]*scale
            tempPos = (tempX,tempY,self.cameraZ)
            # print("cameraPose")
            # print(tempPos)
            cVectorList.append(tempPos)
        return cVectorList

    def compute_wPose(self,cVectorList):
        length = len(cVectorList)
        wPoseList = []
        if(self.TFREADY):
            for i in range(length):
                tempWp = PoseStamped()
                tempWp.header.frame_id = "right_hand_camera"
                tempWp.pose.orientation.w = 1.0  # Neutral orientation
                tempWp.pose.position.x = cVectorList[i][0]
                tempWp.pose.position.y = cVectorList[i][1]
                tempWp.pose.position.z = cVectorList[i][2]
                p_in_base = self.tf_listener_.transformPose("base", tempWp)
                # print("Position of the point in the robot base:")
                # print(p_in_base)
                wPoseList.append(p_in_base)
        return wPoseList

    def cameraInfo_callback(self,data):
        if (not self.PMINIT):
            self.pm.fromCameraInfo(data)
            self.PMINIT = True

def ImageLoc_callback(loc):
    global cubeposclass
    global worldLocation_publisher
    print(loc)
    wLoc = Location()
    uVectorList = cubeposclass.compute_uVector(loc)
    #print('unit Vector')
    #print(uVectorList)
    cVectorList = cubeposclass.compute_cVector(uVectorList)
    world_pos = cubeposclass.compute_wPose(cVectorList)
    print(world_pos)
    wLoc.x.append(world_pos[0].pose.position.x-0.245)
    wLoc.y.append( world_pos[0].pose.position.y-0.286)
    print(wLoc)
    worldLocation_publisher.publish(wLoc)
    pass


def main():
    global  cubeposclass
    global worldLocation_publisher
    rospy.init_node('CubePosition_Node', anonymous=True)
    cubeposclass = CubePositionClass()
    worldLocation_publisher = rospy.Publisher('/world_location', Location, queue_size=1)
    rospy.Subscriber('/location', Location, ImageLoc_callback)
    rospy.spin()


if __name__ == '__main__':
    main()