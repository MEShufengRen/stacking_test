#!/usr/bin/env python
from left_moveIt import MoveItPyPlanner_left
# from right_moveIt import MoveItPyPlanner_right
import rospy
import moveit_commander
from stacking_test.srv import IKService
# from geometry_msgs.msg import Pose



def move_arm(req):
    global _myPlanner_left
    print('============ picking')
    # print(req.x)
    # print(req.y)
    _myPlanner_left.left_gripper.open()
    _myPlanner_left.move_robotArm(req.x, req.y, -0.1)
    _myPlanner_left.pick_block(req.x, req.y, -0.2)
    _myPlanner_left.move_robotArm(req.x, req.y, 0.0)
    print('============ moving')
    _myPlanner_left.move_robotArm(0.4, 0.0, 0.0)

    print('============ placing')
    _myPlanner_left.place_block(0.4, 0.0, -0.2)
    _myPlanner_left.move_robotArm(0.4, 0.0, 0.0)

    _myPlanner_left.group.stop()
    _myPlanner_left.group.clear_pose_targets()
    flag = 'T'
    return flag


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
