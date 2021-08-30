#!/usr/bin/env python

import sys
import select, termios, tty
import rospy
import moveit_commander
import tf

usage = """
Control the position of an end effector
---------------------------
j/l : left/right
i/k : forward/backward
p/; : up/down
x   : reset all joints on arms to zero
e/r : raw -/+
d/f : pitch -/+
c/v : yaw  -/+
s   : setting x y z R P Y
<space> : print current pose
<CTRL-C>: quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def set_arm_pose(group, pose, effector):
    group.set_pose_target(pose, effector)
    plan = group.plan()
    if len(plan.joint_trajectory.points) > 0:
        group.execute(plan, wait=True)
        return True
    else:
        print 'no plan result'
        return False

def reset_arm(group):
    joints = {}
    joints["xmate_joint_1"] = 0.
    joints["xmate_joint_2"] = 0.
    joints["xmate_joint_3"] = 0.
    joints["xmate_joint_4"] = 0.
    joints["xmate_joint_5"] = 0.
    joints["xmate_joint_6"] = 0.
    group.set_joint_value_target(joints)
    plan = group.plan()
    if len(plan.joint_trajectory.points) > 0:
        group.execute(plan, wait=True)
        return True
    else:
        return False

def print_pose(pose):
    q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)
    print '%s: position (%.2f %.2f %.2f) orientation (%.2f %.2f %.2f %.2f) RPY (%.2f %.2f %.2f)' % \
        (effector, pose.position.x, pose.position.y, pose.position.z, \
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, \
        rpy[0], rpy[1], rpy[2])

if __name__=="__main__":
    effector = sys.argv[1] if len(sys.argv) > 1 else 'rokae_link7'

    settings = termios.tcgetattr(sys.stdin)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('teleop_end_effector', anonymous=True)
    #group = moveit_commander.MoveGroupCommander("xarm6")
    group = moveit_commander.MoveGroupCommander("arm")
    group.set_planner_id("RRTConnectkConfigDefault")

    print usage
    ee_pose = group.get_current_pose(effector).pose
    print_pose(ee_pose)
    while(1):
            key = getKey()
            #if key in moveBindings.keys():
            q = (ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w)
            rpy = tf.transformations.euler_from_quaternion(q)
            if key == ' ' :
                ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'c':
                print 'Y-'
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]-0.2)
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'v':
                print 'Y+'
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]+0.2)
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'd':
                print 'P-'
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1]-0.2, rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                print_pose(ee_pose)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'f':
                print 'P+'
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1]+0.2, rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'e':
                print 'R-'
                q = tf.transformations.quaternion_from_euler(rpy[0]-0.2, rpy[1], rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'r':
                print 'R+'
                q = tf.transformations.quaternion_from_euler(rpy[0]+0.2, rpy[1], rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'p':
                print 'z+'
                ee_pose.position.z += 0.05
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== ';':
                print 'z-'
                ee_pose.position.z -= 0.05
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'l':
                print 'y-'
                ee_pose.position.y -= 0.05
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'j':
                print 'y+'
                ee_pose.position.y += 0.05
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'i':
                print 'x+'
                ee_pose.position.x += 0.05
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'k':
                print 'x-'
                ee_pose.position.x -= 0.05
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'x':
                print 'reset'
                reset_arm(group)
                ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key == 's':
                input_str = raw_input("please input x y z R P Y:")
                val_str = input_str.split()
                if len(val_str) <> 6:
                    print 'incorrect input'
                else:
                    ee_pose.position.x = float(val_str[0])
                    ee_pose.position.y = float(val_str[1])
                    ee_pose.position.z = float(val_str[2])
                    q = tf.transformations.quaternion_from_euler(float(val_str[3]),
                                                                 float(val_str[4]),
                                                                 float(val_str[5]))
                    ee_pose.orientation.x = q[0]
                    ee_pose.orientation.y = q[1]
                    ee_pose.orientation.z = q[2]
                    ee_pose.orientation.w = q[3]
                    if not set_arm_pose(group, ee_pose, effector):
                        ee_pose = group.get_current_pose(effector).pose
                    print_pose(ee_pose)

            elif (key == '\x03'):
                break

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
