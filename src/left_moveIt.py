#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
import copy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class MoveItPyPlanner_left(object):
  def __init__(self):
    super(MoveItPyPlanner_left, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()
    self.group_names = self.robot.get_group_names()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group_name = "left_arm"
    self.group = moveit_commander.MoveGroupCommander(self.group_name)
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    self.left_gripper = baxter_interface.Gripper('left')
    self.box_name = ''
    self.eef_link = self.group.get_end_effector_link()

  def CartesianTrajectory(self, x, y, z):
    self.group.set_start_state_to_current_state()
    fraction = 0.0
    max_attempts = 100
    attempts = 0
    waypoints = []
    waypoints.append(self.group.get_current_pose().pose)
    wpose = geometry_msgs.msg.Pose()
    dx = (x - self.group.get_current_pose().pose.position.x) / 30
    dy = (y - self.group.get_current_pose().pose.position.y) / 30
    dz = (z - self.group.get_current_pose().pose.position.z) / 30
    for i in range(30):
      wpose.orientation.x = 1.0
      wpose.position.x = waypoints[i].position.x + dx
      wpose.position.y = waypoints[i].position.y + dy
      wpose.position.z = waypoints[i].position.z + dz
      waypoints.append(copy.deepcopy(wpose))
    while fraction < 1.0 and attempts < max_attempts:
      (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
      attempts += 1
      if attempts % 10 == 0:
        rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
    if fraction == 1.0:
      rospy.loginfo("Path computed successfully. Moving the arm.")
      self.group.execute(plan)
      return
    else:
      rospy.logerr("Could not find valid cartesian path for circle")
    return EmptyResponse()


  def pick_block(self, x, y, z):
    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.x = 1
    # pose_target.position.x = x
    # pose_target.position.y = y
    # pose_target.position.z = z
    # self.group.set_pose_target(pose_target)
    # plan = self.group.plan()
    # self.group.go(wait=True)
    # rospy.sleep(1)
    print("============ Generating block pick plan")
    self.CartesianTrajectory(x, y, z)
    self.left_gripper.close()
    rospy.sleep(1)
    self.group.stop()
    self.group.clear_pose_targets()

  def place_block(self, goal_x, goal_y, goal_z):
    print("============ Generating block placement plan")
    self.CartesianTrajectory(goal_x, goal_y, goal_z)
    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.x = 1
    # pose_target.position.x = goal_x
    # pose_target.position.y = goal_y
    # pose_target.position.z = goal_z
    # self.group.set_pose_target(pose_target)
    # plan = self.group.plan()
    # self.group.go(wait=True)
    rospy.sleep(1)
    self.left_gripper.open()
    rospy.sleep(1)
    self.group.stop()
    self.group.clear_pose_targets()

  def move_robotArm(self, x, y, z):
    print("============ Generating moveArm plan")
    self.CartesianTrajectory(x, y, z)
    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.x = 1
    # pose_target.position.x = x
    # pose_target.position.y = y
    # pose_target.position.z = z
    # self.group.set_pose_target(pose_target)
    # plan = self.group.plan()
    # self.group.go(wait=True)
    rospy.sleep(1)
    self.group.stop()
    self.group.clear_pose_targets()

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in scene.get_known_object_names()

      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()

    return False

  def add_box(self, box_name, frame_id, x, y, z, _size, timeout=4):
    self.box_name = box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = frame_id
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z
    scene.add_box(box_name, box_pose, size=(_size, _size, _size))

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_box(self, box_name, hand_name, timeout=4):
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    grasping_group = hand_name
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, box_name, timeout=4):
    scene = self.scene
    eef_link = self.eef_link
    scene.remove_attached_object(eef_link, name=box_name)
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, remove_name, timeout=4):
    box_name = remove_name
    scene = self.scene
    scene.remove_world_object(box_name)
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)