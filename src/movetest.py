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

def main():
  rospy.init_node('moveItPyPlanner', anonymous=True)
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
    _myPlanner_right.group.set_goal_position_tolerance(0.01)
    _myPlanner_right.group.set_goal_orientation_tolerance(0.01)
    #
    # _myPlanner_left.group.set_joint_value_target([0.094, -0.873, -0.907, 1.456, 0.564, 1.303, 3.034])
    # _myPlanner_left.group.go()
    # _myPlanner_right.group.set_joint_value_target([0.094, -0.873, -0.907, 1.456, 0.564, 1.303, 3.034])
    # _myPlanner_right.group.go()
    # rospy.sleep(3)
    _myPlanner_left.move_robotArm(0.756, -0.038, -0.17)

    # a=_myPlanner_left.group.get_current_joint_values()
    # print(a)
    box_pose=[[0.4,0.2,-0.15],[0.4,-0.2,-0.15]]

    # for i in box_pose:
    #     left_pose=_myPlanner_left.group.get_current_pose().pose
    #     right_pose=_myPlanner_right.group.get_current_pose().pose
    #     left_priority=math.sqrt((left_pose.position.x-i[0])**2+(left_pose.position.y-i[1])**2+(left_pose.position.z-i[2])**2)
    #     right_priority=math.sqrt((right_pose.position.x-i[0])**2+(right_pose.position.y-i[1])**2+(right_pose.position.z-i[2])**2)
    #     if left_priority<right_priority:
    #         print('============ picking')
    #         # _myPlanner_left.pick_block(0.6, 0.5, 0.3)
    #         _myPlanner_left.pick_block(i[0],i[1],i[2])
    #
    #         print('============ moving')
    #         # _myPlanner_left.move_robotArm(0.6, 0.4, 0.3)
    #         _myPlanner_left.move_robotArm(0.4, 0.0, 0)
    #
    #         print('============ placing')
    #         # _myPlanner_left.place_block(0.7, 0.5, 0.3)
    #         _myPlanner_left.place_block(0.4, 0, -0.15)
    #         print('============ moving')
    #         # _myPlanner_left.move_robotArm(0.6, 0.4, 0.3)
    #         _myPlanner_left.move_robotArm(i[0],i[1],i[2])
    #         print("============ STOPPING")
    #     else:
    #         print('============ picking')
    #         # _myPlanner_left.pick_block(0.6, 0.5, 0.3)
    #         _myPlanner_right.pick_block(i[0],i[1],i[2])
    #
    #         print('============ moving')
    #         # _myPlanner_left.move_robotArm(0.6, 0.4, 0.3)
    #         _myPlanner_right.move_robotArm(0.4, 0.0, 0)
    #
    #         print('============ placing')
    #         # _myPlanner_right.place_block(0.7, 0.5, 0.3)
    #         _myPlanner_right.place_block(0.4, 0, -0.15)
    #         print('============ moving')
    #         # _myPlanner_right.move_robotArm(0.6, 0.4, 0.3)
    #         _myPlanner_right.move_robotArm(i[0],i[1],i[2])
    #         print("============ STOPPING")
  ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
  except(rospy.ROSInterruptException):
    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    return


if __name__ == '__main__':
    main()
