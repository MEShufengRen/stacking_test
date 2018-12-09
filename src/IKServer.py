#!/usr/bin/env python
from left_moveIt import MoveItPyPlanner_left
# from right_moveIt import MoveItPyPlanner_right
import rospy
import moveit_commander
from stacking_test.srv import IKService
from actionlib_msgs.msg import GoalStatusArray
import math as mt
# from geometry_msgs.msg import Pose



def move_arm(req):
    global _myPlanner_left
    # global statusFlag
    flag = 'F'
    _myPlanner_left.left_gripper.open()
    print('============ picking')
    _myPlanner_left.move_robotArm(req.x, req.y, 0)
    _myPlanner_left.pick_block(req.x, req.y, -0.17)
    statusFlag = judgesuccess(req.x, req.y, -0.17)
    if (statusFlag == 1):
        print('============ moving')
        _myPlanner_left.move_robotArm(req.x, req.y, 0.0)
        _myPlanner_left.move_robotArm(0.4, 0.4, 0.0)
        statusFlag = judgesuccess(0.4, 0.4, 0.0)
        if (statusFlag == 1):
            print('============ placing')
            _myPlanner_left.place_block(0.4, 0.4, -0.17)
            _myPlanner_left.move_robotArm(0.4, 0.4, 0.0)
            statusFlag = judgesuccess(0.4, 0.4, 0.0)
            if (statusFlag == 1):
                flag = 'T'
    _myPlanner_left.group.stop()
    _myPlanner_left.group.clear_pose_targets()
    return flag


def callback(data):
    # print('callback')
    # print(data.status_list[-1].status)
    global statusFlag
    if data.status_list[-1].status==3:
        statusFlag = 1
    else:
        statusFlag = 2


def judgesuccess(X,Y,Z):
    statusflag = 2
    global _myPlanner_left
    x = _myPlanner_left.group.get_current_pose().pose.position.x
    y = _myPlanner_left.group.get_current_pose().pose.position.y
    z = _myPlanner_left.group.get_current_pose().pose.position.z
    if mt.sqrt((X-x)**2+(Y-y)**2+(Z-z)**2)<0.02:
        statusflag = 1
    return statusflag


def main():
    rospy.init_node('moveItPyPlanner', anonymous=True)
    rate = rospy.Rate(10)
    # cameraz_pub = rospy.Publisher('cameraz', Pose, queue_size=10)
    # _myPlanner_right = MoveItPyPlanner_right()
    # _myPlanner_right.move_robotArm(0.6, -0.1, 0.25)
    # camera_pose = _myPlanner_right.group.get_current_pose().pose
    # print(camera_pose)
    # cameraz_pub.publish(camera_pose)
    global _myPlanner_left
    global statusFlag
    statusFlag = 0
    #finish_pub = rospy.Publisher('/PickFinish', int, queue_size=1)
    rospy.Subscriber('/move_group/status', GoalStatusArray, callback)
    _myPlanner_left = MoveItPyPlanner_left()
    _myPlanner_left.group.clear_pose_targets()
    _myPlanner_left.group.set_pose_reference_frame('base')
    _myPlanner_left.group.allow_replanning(True)
    _myPlanner_left.group.set_goal_position_tolerance(0.005)
    _myPlanner_left.group.set_goal_orientation_tolerance(0.005)
    _myPlanner_left.group.set_planning_time(5.0)
    _myPlanner_left.group.set_joint_value_target([0.094, -0.873, -0.907, 1.456, 0.564, 1.303, 3.034])
    _myPlanner_left.group.go()
    rospy.Service('move_arm', IKService, move_arm)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        moveit_commander.roscpp_shutdown()
        pass
