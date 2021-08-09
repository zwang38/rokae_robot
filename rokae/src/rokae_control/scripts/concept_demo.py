#!/usr/bin/env python
import sys, random, copy
import rospy, tf, rospkg
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from std_msgs.msg import *
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def robot_move(group, x, y, z, R, P, Y):
    # moveit_commander.roscpp_initialize(sys.argv)
    # robot = moveit_commander.RobotCommander()

    group.clear_pose_targets()
    quat = tf.transformations.quaternion_from_euler(R, P, Y)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]
    group.set_pose_target(pose_target)

    plan = group.plan()
    group.execute(plan, wait=True)
    print(("test %f,%f,%f,%f,%f,%f")%(x,y,z,R,P,Y))


if __name__ == "__main__":
    # First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_sorting', anonymous=True)
    # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    # the robot:
    robot = moveit_commander.RobotCommander()
    # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    # to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    group_name1 = "arm"
    group1 = moveit_commander.MoveGroupCommander(group_name1)
    group1.set_planner_id("RRTConnect")

    #above the bolt 1
    #robot_move(group1, -0.057262, 0.038543, 1.1700, 3.14, 0, 0)

    robot_move(group1, -0.057262, 0.038543, 1.3700, 3.14, 0, 0)

    rospy.sleep(5)

    #socketed posistion
    robot_move(group1, -0.057262, 0.038543, 1.140, 3.14, 0, 0)

