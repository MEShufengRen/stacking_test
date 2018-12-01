#!/usr/bin/env python


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

block1_flag = False
block2_flag = False
block3_flag = False
sleep_flag = True
xpos_up = 0
ypos_up = 0

class MoveItPyPlanner(object):
  def __init__(self):
    super(MoveItPyPlanner, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveItPyPlanner', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    self.robot = moveit_commander.RobotCommander()
    self.group_names = self.robot.get_group_names()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    self.group_name = "left_arm"
    self.group = moveit_commander.MoveGroupCommander(self.group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ##get left gripper of the baxter
    self.left_gripper = baxter_interface.Gripper('left')

    ##related boxes
    self.box_name = ''

    self.eef_link = self.group.get_end_effector_link()

    rospy.sleep(2)

  def pick_block(self,x,y,z):
    print("============ Generating Block Pick Up Plan")
    pose_target = geometry_msgs.msg.Pose()

    pose_target.orientation.x = 1
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    self.group.set_pose_target(pose_target)
    plan = self.group.plan()
    self.group.go(wait=True)
    rospy.sleep(2)

    # Close Baxter's left gripper
    self.left_gripper.close()
    rospy.sleep(2)

    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    #current_pose = self.group.get_current_pose().pose

    return

  def place_block(self,goal_x,goal_y,goal_z):
    print("============ Generating block placement plan")
    pose_target = geometry_msgs.msg.Pose()


    pose_target.orientation.x = 1
    pose_target.position.x = goal_x
    pose_target.position.y = goal_y
    pose_target.position.z = goal_z
    self.group.set_pose_target(pose_target)
    plan = self.group.plan()
    self.group.go(wait=True)
    rospy.sleep(2)

    # Close Baxter's left gripper
    self.left_gripper.open()
    rospy.sleep(2)

    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    #current_pose = self.group.get_current_pose().pose

    return

  def move_robotArm(self,x,y,z):
    print("============ Generating waypoint plan")
    pose_target = geometry_msgs.msg.Pose()

    pose_target.orientation.x = 1
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    self.group.set_pose_target(pose_target)
    plan = self.group.plan()
    self.group.go(wait=True)
    rospy.sleep(2)

    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    #current_pose = self.group.get_current_pose().pose

    return

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_box(self, box_name,frame_id,x,y,z,_size,timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    self.box_name = box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = frame_id
    box_pose.pose.orientation.w = 1.0
    #box_pose.pose.position.z = 0.07 # slightly above the end effector
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z
    scene.add_box(box_name, box_pose, size=(_size, _size, _size))

    ## END_SUB_TUTORIAL
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, box_name,hand_name,timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    #grasping_group = 'left_hand'
    grasping_group = hand_name
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, box_name,timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    #box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, removedbox_name,timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = removedbox_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    _myPlanner = MoveItPyPlanner()

    #current_pose =_myPlanner.group.get_current_pose()
    _myPlanner.add_box('box','left_gripper',0,0,0.07,0.1)
    #_myPlanner.attach_box('box','left_hand')

    rospy.sleep(20)

    #_myPlanner.detach_box('box')
    _myPlanner.remove_box('box')

    # print('============ picking')
    # _myPlanner.pick_block(0.6, 0.5, 0.3)
    #
    # print('============ moving')
    # _myPlanner.move_robotArm(0.6, 0.4, 0.3)
    #
    # print('============ placing')
    # _myPlanner.place_block(0.7, 0.5, 0.3)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    ## END_TUTORIAL
    print("============ STOPPING")

  except(rospy.ROSInterruptException):
    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    return

if __name__ == '__main__':
    main()
