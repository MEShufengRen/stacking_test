#!/usr/bin/env python
from left_moveIt import MoveItPyPlanner_left
from right_moveIt import MoveItPyPlanner_right
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
from baxter_interface import Gripper
from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from baxter_core_msgs.msg import DigitalIOState
from moveit_commander import MoveGroupCommander
import math as math
from sensor_msgs.msg import Range


def callback(data):
  global zdistance
  zdistance = data.range

def main():
  rospy.init_node('moveItPyPlanner', anonymous=True)
  rospy.Subscriber('/robot/range/left_hand_range/state', Range, callback)
  try:
    _myPlanner_left = MoveItPyPlanner_left()
    _myPlanner_left.group.clear_pose_targets()
    _myPlanner_left.group.set_pose_reference_frame('base')
    _myPlanner_left.group.allow_replanning(True)
    _myPlanner_left.group.set_goal_position_tolerance(0.01)
    _myPlanner_left.group.set_goal_orientation_tolerance(0.01)


    _myPlanner_right = MoveItPyPlanner_right()
    _myPlanner_right.group.clear_pose_targets()
    _myPlanner_right.group.set_pose_reference_frame('base')
    _myPlanner_right.group.allow_replanning(True)
    _myPlanner_right.group.set_goal_position_tolerance(0.005)
    _myPlanner_right.group.set_goal_orientation_tolerance(0.005)
    global zdistance
    print(zdistance)
    currentz = _myPlanner_left.group.get_current_pose().pose.position.z
    currentx = _myPlanner_left.group.get_current_pose().pose.position.x
    currenty = _myPlanner_left.group.get_current_pose().pose.position.y
    print(currentz)
    _myPlanner_left.move_robotArm(0.4, 0.4, -0.22)
    moveit_commander.roscpp_shutdown()
  except(rospy.ROSInterruptException):
    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    return


if __name__ == '__main__':
    main()
