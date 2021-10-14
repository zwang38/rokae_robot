#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import threading

import tf
import sys
import cv2
import time
import rospy
import random
import pprint
import image_geometry
import message_filters
import numpy as np
from itertools import chain
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from tf import TransformListener, transformations
import testmotion
# from  bolt_position_detector
import templateMatching
import copy
import moveit_commander


# from PIL import Image,ImageDraw
# import numpy as np
from prim_aim_target import PrimAimTarget
from prim_clear_obstacle import PrimClearObstacle
from prim_insert import PrimInsert
from prim_move import PrimMove
import tf2_ros
import geometry_msgs.msg

import socket
import pickle
import struct


class NSPlanner:
    def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic):

        self.camera_name = camera_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.bolt_trans_topic = '/NSPlanner/bolt_trans'

        self.pose = None

        self.marker_pub = rospy.Publisher(
            'visualization_marker', Marker, queue_size=10)

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
        self.sub_rgb = message_filters.Subscriber(
            rgb_topic, Image, queue_size=q)
        self.sub_depth = message_filters.Subscriber(
            depth_topic, Image, queue_size=q)
        self.sub_camera_info = rospy.Subscriber(
            camera_info_topic, CameraInfo, self.cam_info_cb)
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
        self.bolt_pose = None
        self.all_infos_lock = threading.Lock()
        self.prim_thread = threading.Thread(target=self.do_action)
        self.prim_execution = True
        self.prim_thread.start()

        ip_port = ('127.0.0.1', 5050)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(ip_port)

        # self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # ip_port_class = ('127.0.0.2', 6050)
        # self.server.bind(ip_port_class)
        # self.server.listen(5)

    def pack_image(self, frame):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        result, frame = cv2.imencode('.jpg', frame, encode_param)
        data = pickle.dumps(frame, 0)
        size = len(data)
        packed = struct.pack(">L", size) + data
        self.sock.sendall(packed)
        print("send all finished")
        # return packed, data, size
        # -------- 接收server端发送的结果文件数据 --------
        while True:
            data = self.sock.recv(4096)
            if data:
                result = pickle.loads(data)
                print(result)
                return result

    def is_stoping(self):
        return self.action == 'end'

    def get_bolt_pose(self):
        # only call for get experiment result
        return self.bolt_pose

    def plan(self):
        prev_action = self.action
        # print(self.ret_dict)
        if 'success' in self.ret_dict.keys() and self.ret_dict['success'] is True:
            if self.action == 'start':
                self.action = 'move'
            elif self.action == 'move':
                self.action = 'aim'
                is_state = self.pack_image(self.all_infos['rgb_img'])
                # self.sock.close()

            elif self.action == 'aim':
                self.action = 'clear'
            elif self.action == 'clear':
                self.action = 'end'
            # skip insert prim for testing
            #     self.action = 'insert'
            # elif self.action == 'insert':
            #     self.action = 'end'
        print("%s --> %s" % (prev_action, self.action))

    def start(self,  pose):
        if self.action != 'end':
            print("Please start after previous task was done!")
            return False
        else:
            self.ret_dict['coarse_pose'] = pose
            self.ret_dict['success'] = True
            self.action = 'start'
            self.bolt_pose = pose
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
                    if 'bolt_pose' in self.ret_dict.keys():
                        self.bolt_pose = self.ret_dict['bolt_pose']
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
            # rospy.loginfo('receiving image')
            if self.all_infos_lock.acquire():
                self.all_infos = {'rgb_img': img, 'depth_img': depth_img,
                                  'camera_model': self.camera_model, 'timestamp': ts}
                self.all_infos_lock.release()

        except Exception, err:
            print("exception happen in message call back:", err)

    def __del__(self):
        self.prim_execution = False
        self.prim_thread.join()


if __name__ == '__main__':

    # 加载电池包，不加载直接回车
    # testmotion.load_battery()
    # testmotion.robot_position(0, 0, 1.5)
    try:
        rospy.init_node('nsplanner-moveit', anonymous=True)

        planner = NSPlanner('camera', '/camera/color/image_raw',
                            '/camera/depth/image_raw', '/camera/color/camera_info')

        quat = tf.transformations.quaternion_from_euler(-3.14, 0, 0)
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = 0
        pose_target.position.y = 0
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
