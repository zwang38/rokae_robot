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
# import nsplanner
# import matplotlib.mlab as mlab
import numpy as np
# import matplotlib.pyplot as plt
import math
# import mpl_toolkits.mplot3d
from numpy import random

# from PIL import Image,ImageDraw
# import numpy as np
from prim_aim_target import PrimAimTarget
from prim_clear_obstacle import PrimClearObstacle
from prim_insert import PrimInsert
from prim_move import PrimMove
import tf2_ros
import geometry_msgs.msg
from visualization_msgs.msg import Marker
import threading


class NSPlanner:
    def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic):

        self.camera_name = camera_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.bolt_trans_topic = '/NSPlanner/bolt_trans'

        self.pose = None

        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
        # cv2.setMouseCallback("Image window", self.mouse_callback)

        self.br = tf2_ros.TransformBroadcaster()

        # Have we recieved camera_info and image yet?
        self.ready_ = False

        self.bridge = CvBridge()

        self.camera_model = image_geometry.PinholeCameraModel()
        rospy.loginfo(
            'Camera {} initialised, {}, , {}'.format(self.camera_name, rgb_topic, depth_topic, camera_info_topic))
        print('')

        q = 1
        self.sub_rgb = message_filters.Subscriber(rgb_topic, Image, queue_size=q)
        self.sub_depth = message_filters.Subscriber(depth_topic, Image, queue_size=q)
        self.sub_camera_info = rospy.Subscriber(camera_info_topic, CameraInfo, self.cam_info_cb)
        self.camera_model_ready = False
        self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth],
                                                               queue_size=30, slop=0.2)

        self.tss.registerCallback(self.callback)


        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.group.set_planner_id("RRTConnectkConfigDefault")


        self.aim_target_prim = PrimAimTarget(self.group)
        self.clear_obstacle_prim = PrimClearObstacle(self.group)
        self.insert_prim = PrimInsert(self.group)
        self.move_prim = PrimMove(self.group)
        self.prims = {'aim': self.aim_target_prim,
                      'clear': self.clear_obstacle_prim,
                      'insert': self.insert_prim,
                      'move': self.move_prim}
        self.action = 'end'
        self.all_infos = {}
        self.ret_dict = {}
        self.ret_dict  ['success']=True
        self.all_infos_lock = threading.Lock()
        self.prim_thread = threading.Thread(target=self.do_action)
        self.prim_execution = True
        self.prim_thread.start()


    def plan(self):
        try:
            prev_action = self.action
            print('is start ?,{0}'.format(self.ret_dict))
            if self.ret_dict['success'] is True:
                if self.action == 'start':
                    self.action = 'move'
                elif self.action == 'move':
                    self.action = 'aim'
                elif self.action == 'aim':
                    self.action = 'clear'
                elif self.action == 'clear':
                    self.action = 'insert'
                elif self.action == 'insert':
                    self.action = 'end'
            print("%s --> %s"%(prev_action,self.action))
        except Exception, err:
            print("exception plan, Don't care it will run again :", err)
            
    def start(self,  pose):
        if self.action != 'end':
            print("Please start after previous task was done!")
            return False
        else:
            self.ret_dict['coarse_pose'] = pose
            self.ret_dict['success'] = True
            self.action = 'start'
            return True

    def do_action(self):
        while self.prim_execution:
            self.plan()
            if self.action == 'end':
                rospy.sleep(1)
                continue
            if self.all_infos_lock.acquire():
                infos = copy.deepcopy(self.all_infos)
                self.all_infos.clear()
                self.all_infos_lock.release()
                if self.action in self.prims.keys():
                    prim = self.prims[self.action]
                    self.ret_dict = prim.action(infos, self.ret_dict)
            rospy.sleep(1)

    def cam_info_cb(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.camera_model_ready = True
        self.sub_camera_info.unregister()

    def callback(self, rgb_msg, depth_msg):
        try:
            if not self.camera_model_ready:
                print("camera info is not ready")
                return
            img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
            ts = rospy.Time.now()
            #rospy.loginfo('receiving image')
            if self.all_infos_lock.acquire():
                self.all_infos = {'rgb_img': img, 'depth_img': depth_img,
                                  'camera_model': self.camera_model, 'timestamp': ts}
                self.all_infos_lock.release()

        except Exception, err:
            print("exception happen in message call back:", err)

    def __del__(self):
        self.prim_execution = False
        self.prim_thread.join()
        
        
def get_gazebo_model_pose():
    parts_pose = []
    model_pose = rospy.wait_for_message("gazebo/model_states", ModelStates)
    # for count in range(len(model_pose.name)-1):
    if len(model_pose.name) > 2:
        current_product = len(model_pose.name)-1
        name = model_pose.name[current_product]
        x = model_pose.pose[current_product].position.x
        y = model_pose.pose[current_product].position.y

        return x,  y
    else:
        return 0, 0


def writelogs(write_data):
    # write_data.sort(key=takeSecond)
    # 打开文件
    file_name = 'random_deviation.txt'

    fo = open(file_name, 'w')
    print "文件名为: ", fo.name
    for every in write_data:
        fo.write(every + "\n")

    fo.close()


def move_robot_nsplanner( x_offset, y_offset):
    x_pos_battery, y_pos_battery = get_gazebo_model_pose()

    quat = tf.transformations.quaternion_from_euler(-3.14, 0, 0)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x_pos_battery + x_bolt + x_offset
    pose_target.position.y = y_pos_battery + y_bolt + y_offset
    pose_target.position.z = z_bolt
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]
    print('we have started.')
    return planner.start(pose_target)


if __name__ == "__main__":
    rospy.init_node('nsplanner-moveit', anonymous=True)


    planner = NSPlanner('camera', '/camera/color/image_raw',
                                   '/camera/depth/image_raw', '/camera/color/camera_info')

    x_bolt = -0.057323   # 数值大，向下
    y_bolt = 0.03838  # 数值大，向左
    z_bolt = 1.3

    deviation = 0.03
    mu = 0
    datasets = []
    datasets.append('x,y,semidiameter,normal_success,nsplanner_success')
    # datasets = [0.01, 0.02, 0.03, 0.04, 0.05]
    for step in range(1, 11):
        is_probability = False
        is_success_nsplanner = False
        current_sigma = round(float(step)/100, 2)
        semidiameter = np.random.normal(loc=mu, scale=current_sigma, size=10)
        angle = np.random.randint(360, size=10)

        for number in range(len(semidiameter)):
            x_current = abs(semidiameter[number]) * \
                math.cos(2 * math.pi * angle[number] / 360)
            y_current = abs(semidiameter[number]) * \
                math.sin(2 * math.pi * angle[number] / 360)

            if abs(semidiameter[number]) <= deviation:
                is_probability = True


            is_success_nsplanner = move_robot_nsplanner( x_current, y_current)
            
            datasets.append('{},{},{},{}'.format(x_current, y_current,
                            semidiameter[number], is_probability, is_success_nsplanner))

            is_probability = False

    writelogs(datasets)
    while not rospy.is_shutdown():
        rospy.spin()
    # z_bolt=1.1815
    # try:
    #     for data in datasets:

    #         # rospy.init_node('nsplanner-moveit', anonymous=True)

    #         # planner = nsplanner. NSPlanner('camera', '/camera/color/image_raw',
    #         #                                '/camera/depth/image_raw', '/camera/color/camera_info')

    #         x_pos_battery, y_pos_battery = get_gazebo_model_pose()

    #         quat = tf.transformations.quaternion_from_euler(-3.14, 0, 0)
    #         pose_target = geometry_msgs.msg.Pose()
    #         pose_target.position.x = x_pos_battery + x_bolt + data
    #         pose_target.position.y = y_pos_battery + y_bolt
    #         pose_target.position.z = z_bolt
    #         pose_target.orientation.x = quat[0]
    #         pose_target.orientation.y = quat[1]
    #         pose_target.orientation.z = quat[2]
    #         pose_target.orientation.w = quat[3]
    #         planner.start(pose_target)

    #         while not rospy.is_shutdown():
    #             rospy.spin()

    # except rospy.ROSInterruptException:
    #     print("Shutting down")
    #     cv2.destroyAllWindows()
