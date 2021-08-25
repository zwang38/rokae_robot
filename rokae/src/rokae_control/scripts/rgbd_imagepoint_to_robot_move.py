#!/usr/bin/python
# -*- coding: utf-8 -*-

# tf2_ros_example.py: example showing how to use tf2_ros API
# Author: Ravi Joshi
# Date: 2018/12/6

# import modules
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped


import traceback
import rospy
import tf
import message_filters
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image

from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import bolt_position_detector
import cv2

import testmotion




def cam_info_cb(msg):
  global cam_model
  print('got cam model')

  cam_model = PinholeCameraModel()
  cam_model.fromCameraInfo(msg)
  cam_sub.unregister()

def transform_point(src_frame, tgt_frame, pose_pt, ts):
  '''
  transform pose of given point from 'src_frame' to 'tgt_frame'
  '''

  ps_src = PointStamped()
  try:

    tf_listener.waitForTransform(tgt_frame, src_frame, ts, rospy.Duration(0.2))

    ps_src.header.frame_id = src_frame
    ps_src.header.stamp = ts
    ps_src.point = pose_pt

    ps_tgt = tf_listener.transformPoint(tgt_frame, ps_src)

    return ps_tgt.point
  except:
    traceback.print_exc()
    rospy.signal_shutdown('')



# def bolt_callback(self, rgb_msg, depth_msg, camera_info_msg):
#     self.camera_model.fromCameraInfo(camera_info_msg)
#     img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
#     depth_32FC1 = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
#     self.latest_depth_32FC1 = depth_32FC1.copy()

def bolt_callback(rgb_msg, depth_msg):
    print("bolt_callback")

    rgb_img =  bridge.imgmsg_to_cv2(rgb_msg,'16UC1')

    cv2.imshow("Image window", rgb_img)

    depth_img = bridge.imgmsg_to_cv2(depth_msg,'rgb8')

    # use canny detect bolt
    #....
    #end detect

    x,y,w,h=bolt_position_detector.detection_position(rgb_img)

    # x = 10
    # y = 10
    # w = 10
    # h = 10

    c_x = x + int(w/2)
    c_y = y + int(h/2)
    d = 1.8#depth_img[c_y][c_x]/1000.0  # in meters

    coord_x = (c_x - cam_model.cx())*d*(1.0/cam_model.fx())
    coord_y = (c_y - cam_model.cy())*d*(1.0/cam_model.fy())

    pt = Point()
    pt.x = coord_x
    pt.y = coord_y
    pt.z = d

    source_frame = 'camera_depth_frame'#'image'
    target_frame = 'world' #'world'

    ts = rospy.Time.now()
    pt_world = transform_point(source_frame, target_frame, pt, ts)

    print(("%f, %f, %f")%(pt_world.x, pt_world.y, pt_world.z))
    testmotion.robot_position(0.8, 0.9)



if __name__ == '__main__':



  rospy.init_node('bolt_pose')

  rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw')
  depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
  cam_info_topic = rospy.get_param('~cam_info_topic', '/camera/color/camera_info')
  local_run = rospy.get_param('~local', False)
  hz = rospy.get_param('~hz', 1)


  print('subscribe to {} for rgb image:'.format(rgb_topic))
  rgb_sub = message_filters.Subscriber(rgb_topic, Image)
  print('subscribe to {} for depth image:'.format(depth_topic))
  depth_sub = message_filters.Subscriber(depth_topic, Image)
  bolt_rgb_depth_sub = message_filters.ApproximateTimeSynchronizer([depth_sub, rgb_sub],30,0.2)
  bolt_rgb_depth_sub.registerCallback(bolt_callback)

  print('subscribe to {} for camera info'.format(cam_info_topic))
  cam_sub = rospy.Subscriber(cam_info_topic, CameraInfo, cam_info_cb)

  tf_listener = tf.TransformListener()
  bridge = CvBridge()

  rospy.spin()