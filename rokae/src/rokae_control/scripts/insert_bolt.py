#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import random
import copy
import rospy
import tf
import rospkg
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from std_msgs.msg import *

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import concept_demo
import math
from time import sleep
from gazebo_msgs.srv import DeleteModel
import testmotion

import select
import termios
import tty

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
import image_geometry
import os
import nsplanner


def product_spawn(x_bolt_pos, y_bolt_pos):
    # This function spawn three types parts(screw1, screw2, woodbolt) in gazebo

    rospy.wait_for_service("gazebo/spawn_urdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    rospack = rospkg.RosPack()
    part_pkg = rospack.get_path('cai_env')

    max_count = 3
    max_num = 3

    for count in range(max_count):
        for num in range(0, max_num):
            item_name = "product_{0}_{1}".format(count, num)
            delete_model(item_name)
    # if clear_only:
    #     return
    for count in range(max_count):  # three types products spawn in gazebo
        if (count == 0):
            with open(part_pkg + '/urdf/' + 'block.urdf', "r") as wood1:
                product_xml = wood1.read()
                wood1.close()

        elif (count == 1):
            with open(part_pkg + '/urdf/' + 'screw1.urdf', "r") as screw1:
                product_xml = screw1.read()
                screw1.close()
        else:
            with open(part_pkg + '/urdf/' + 'screw2.urdf', "r") as screw2:
                product_xml = screw2.read()
                screw2.close()

        for num in range(0, max_num):
            print(num)
            x_rand = random.randrange(-20, 20) * 0.01
            y_rand = random.randrange(-20, 20) * 0.01
            R_rand = random.randrange(-314, 314) * 0.01
            P_rand = random.randrange(-314, 314) * 0.01
            Y_rand = random.randrange(-314, 314) * 0.01
            if (num == 0):
                x_rand = x_bolt_pos
                y_rand = y_bolt_pos
                R_rand = 0
                P_rand = 0
                Y_rand = 0
            quat = tf.transformations.quaternion_from_euler(
                1.57, P_rand, Y_rand)
            orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
            item_name = "product_{0}_{1}".format(count, num)
            print("Spawning model:%s, %f, %f, %f",
                  item_name, x_rand, y_rand, 1.235)
            item_pose = Pose(Point(x=x_rand, y=y_rand, z=1.235), orient)
            spawn_model(item_name, product_xml, "", item_pose, "world")
            rospy.sleep(2)


global capture_number


def load_obstacle(x_bolt_pos, y_bolt_pos):

    print('加载障碍物,please input add')
    input_delete = raw_input()

    if input_delete == 'add':
        # concept_demo.product_spawn()
        product_spawn(x_bolt_pos, y_bolt_pos)


def get_key():
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
        print ('no plan result')
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
    q = (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)
    print('%s: position (%.2f %.2f %.2f) orientation (%.2f %.2f %.2f %.2f) RPY (%.2f %.2f %.2f)' %
          (effector, pose.position.x, pose.position.y, pose.position.z,
           pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
           rpy[0], rpy[1], rpy[2]))


def set_align_vertical_capture(ee_pose,   x_bolt, y_bolt, z_bolt):
    x_battery,  y_battery = get_gazebo_model_pose()
    # 相机旋转
    delta_rpy_random = random.randint(-314, 314)
    delta_rpy_random = float(delta_rpy_random)/float(100.0)

    q = (ee_pose.orientation.x, ee_pose.orientation.y,
         ee_pose.orientation.z, ee_pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)

    # Z轴移动范围
    z_hight = random.randint(118, 150)
    z_random_hight = float(z_hight)/float(100.0)

    ee_pose.position.x = x_battery + x_bolt
    ee_pose.position.y = y_battery + y_bolt

    print(' range z  ')
    ee_pose.position.z = z_random_hight
    # ee_pose.position.z = 1.18

    # rpy:变换
    q = tf.transformations.quaternion_from_euler(
        -math.pi, rpy[1], rpy[2]+delta_rpy_random)
    ee_pose.orientation.x = q[0]
    ee_pose.orientation.y = q[1]
    ee_pose.orientation.z = q[2]
    ee_pose.orientation.w = q[3]
    # set_arm_pose(group, ee_pose, effector)
    if set_arm_pose(group, ee_pose, effector):
        camera.set_capture()
    else:
        ee_pose = group.get_current_pose(effector).pose
    print_pose(ee_pose)


def set_shift_vertical(ee_pose,   x_bolt, y_bolt):
    x_battery,  y_battery = get_gazebo_model_pose()

    delta_rpy_random = random.randint(-314, 314)
    delta_rpy_random = float(delta_rpy_random)/float(100.0)
    q = (ee_pose.orientation.x, ee_pose.orientation.y,
         ee_pose.orientation.z, ee_pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)

    x_center_bolt_pos = x_battery+x_bolt
    y_center_bolt_pos = y_battery+y_bolt
    # ee_pose.position.x =x_battery +x_bolt
    # ee_pose.position.y =y_battery +y_bolt

    x_delta = random.randint(-25, 25)
    x_delta_random = float(x_delta)/float(1000.0)

    y_delta = random.randint(-25, 25)
    y_delta_random = float(y_delta)/float(1000.0)

    # z_hight=random.randint(118,150)
    # z_random_hight=float(z_hight)/float(100.0)

    print(x_delta)
    print(y_delta_random)

    ee_pose.position.x = x_delta_random + x_center_bolt_pos

    ee_pose.position.y = y_delta_random+y_center_bolt_pos

    ee_pose.position.z = 118.15

    # rpy:变换
    q = tf.transformations.quaternion_from_euler(
        -math.pi, rpy[1], rpy[2]+delta_rpy_random)
    ee_pose.orientation.x = q[0]
    ee_pose.orientation.y = q[1]
    ee_pose.orientation.z = q[2]
    ee_pose.orientation.w = q[3]

    set_arm_pose(group, ee_pose, effector)

    print_pose(ee_pose)


def get_gazebo_model_pose():
    parts_pose = []
    model_pose = rospy.wait_for_message("gazebo/model_states", ModelStates)
    # for count in range(len(model_pose.name)-1):
    if len(model_pose.name) > 2:
        current_product = len(model_pose.name)-1
        name = model_pose.name[current_product]
        x = model_pose.pose[current_product].position.x
        y = model_pose.pose[current_product].position.y

    # ee_pose = model_pose.pose[current_product].pose

    # parts_pose.append([name, x, y])

        return x,  y
    else:
        return 0, 0


if __name__ == "__main__":

    x_bolt = -0.057323   # 数值大，向下
    y_bolt = 0.03838  # 数值大，向左
    z_bolt = 1.3
    # z_bolt=1.1815

    effector = sys.argv[1] if len(sys.argv) > 1 else 'rokae_link7'

    settings = termios.tcgetattr(sys.stdin)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('teleop_end_effector', anonymous=True)
    #group = moveit_commander.MoveGroupCommander("xarm6")
    group = moveit_commander.MoveGroupCommander("arm")
    group.set_planner_id("RRTConnectkConfigDefault")

    # print (usage)

    ee_pose = group.get_current_pose(effector).pose
    print_pose(ee_pose)

    # x_battery,  y_battery = get_gazebo_model_pose()
    # testmotion.robot_position(
    #     x_battery + x_bolt, y_battery + y_bolt, z_bolt)  # 移动到电池螺栓

    # 加载电池包，不加载直接回车
    # testmotion.load_battery()
    # testmotion.robot_position(0, 0, 1.5)
    try:
        rospy.init_node('nsplanner-moveit', anonymous=True)

        planner = nsplanner. NSPlanner('camera', '/camera/color/image_raw',
                                       '/camera/depth/image_raw', '/camera/color/camera_info')

        quat = tf.transformations.quaternion_from_euler(-3.14, 0, 0)
        pose_target = geometry_msgs.msg.Pose()
        x_battery,  y_battery = get_gazebo_model_pose()

        pose_target.position.x =x_battery +x_bolt 
        pose_target.position.y =y_battery+y_bolt
        pose_target.position.z = 1.3
        pose_target.orientation.x = quat[0]
        pose_target.orientation.y = quat[1]
        pose_target.orientation.z = quat[2]
        pose_target.orientation.w = quat[3]
        planner.start(pose_target)

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        print("Shutting down")
        cv2.destroyAllWindows()

    # print('对齐偏移shift,please input s')
    # input_delete=raw_input()

    # if input_delete=='s':
    #     # product_spawn(x_bolt_pos , y_bolt_pos)
    #     set_shift_vertical(ee_pose ,   x_bolt, y_bolt)
