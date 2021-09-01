#!/usr/bin/python
# -*- coding: utf-8 -*-

# tf2_ros_example.py: example showing how to use tf2_ros API
# Author: Ravi Joshi
# Date: 2018/12/6

# import modules
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped
import numpy as np

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
    rgb_img_path='src/rokae_control/images/rgb_img.jpg'
    depth_img_path='src/rokae_control/images/depth_img.jpg'

    try:
      #RGB图像
      rgb_img =  bridge.imgmsg_to_cv2(rgb_msg, 'bgr8' )
      #深度图像
      depth_img = bridge.imgmsg_to_cv2(depth_msg,  '16UC1')
      cv2.imwrite(rgb_img_path, rgb_img)
      cv2.imwrite(depth_img_path, depth_img) 
      rospy.sleep(5)
    except CvBridgeError as e:
      print(e)
    

    # use canny detect bolt
    x,y,w,h=bolt_position_detector.detection_position(rgb_img_path)

    # clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
    # latest_depth_32FC1=depth_img.copy()
    # #Small ROI around clicked point grows larger if no depth value found
    # for bbox_width in range(20, int(latest_depth_32FC1.shape[0]/3), 5):
    #     tl_x = clamp(x-bbox_width/2, 0, latest_depth_32FC1.shape[0])
    #     br_x = clamp(x+bbox_width/2, 0, latest_depth_32FC1.shape[0])
    #     tl_y = clamp(y-bbox_width/2, 0, latest_depth_32FC1.shape[1])
    #     br_y = clamp(y+bbox_width/2, 0, latest_depth_32FC1.shape[1])
    #     print((x, y), (tl_x, tl_y, br_x, br_y))
    #     roi = latest_depth_32FC1[tl_y:br_y, tl_x:br_x]
    #     depth_distance = np.median(roi)

    c_x = x + int(w/2)
    c_y = y + int(h/2)
    d = depth_img[c_y][c_x]/1000.0  # in meters

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

  rospy.init_node('bolt_pose' ,anonymous=True)

  # 移动机械臂到电池上方，我的电脑老是似乎不会动
  # testmotion.robot_position(0.4,0,1.2)

  bridge = CvBridge()
  # rgb_topic = rospy.get_param('~rgb_topic', '/gloal_camera/color/image_raw')
  # depth_topic = rospy.get_param('~depth_topic', '/gloal_camera/depth/image_raw')
  # cam_info_topic = rospy.get_param('~cam_info_topic', '/gloal_camera/color/camera_info')


  # rate = rospy.Rate(1)
  # rate.sleep()

  # 相机消息订阅，
  rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw')
  depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
  cam_info_topic = rospy.get_param('~cam_info_topic', '/camera/color/camera_info')


  local_run = rospy.get_param('~local', False)
  hz = rospy.get_param('~hz', 1)

  q=25
  rgb_sub = message_filters.Subscriber(rgb_topic, Image , queue_size=q)
  print('subscribe to {} for rgb image:'.format(rgb_topic))

  depth_sub = message_filters.Subscriber(depth_topic, Image, queue_size=q)
  print('subscribe to {} for depth image:'.format(depth_topic))

  bolt_rgb_depth_sub = message_filters.ApproximateTimeSynchronizer([rgb_sub,depth_sub],queue_size=30, slop=0.5)

  cam_sub = rospy.Subscriber(cam_info_topic, CameraInfo, cam_info_cb, queue_size=q)
  print('subscribe to {} for camera info'.format(cam_info_topic))
  

  # 休眠5秒，完成消息订阅
  # rospy.sleep(5)
  bolt_rgb_depth_sub.registerCallback(bolt_callback)

  tf_listener = tf.TransformListener()


  rospy.spin()