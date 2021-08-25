#!/usr/bin/env python
import os
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


class Camera():
    def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic):

        self.camera_name = camera_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic

        self.pose = None

        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
        # cv2.setMouseCallback("Image window", self.mouse_callback)

        self.br = tf.TransformBroadcaster()

        #Have we recieved camera_info and image yet?
        self.ready_ = False

        self.bridge = CvBridge()

        self.camera_model = image_geometry.PinholeCameraModel()
        rospy.loginfo('Camera {} initialised, {}, , {}'.format(self.camera_name, rgb_topic, depth_topic, camera_info_topic))
        print('')

        q=25
        self.sub_rgb = message_filters.Subscriber(rgb_topic, Image, queue_size=q)
        self.sub_depth = message_filters.Subscriber(depth_topic, Image, queue_size=q)
        self.sub_camera_info = message_filters.Subscriber(camera_info_topic, CameraInfo, queue_size=q)
        # self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_camera_info], queue_size=15, slop=0.4)
        self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_camera_info], queue_size=30, slop=0.5)
        #self.tss = message_filters.TimeSynchronizer([sub_rgb], 10)

        self.tss.registerCallback(self.callback)

    def callback(self, rgb_msg, depth_msg, camera_info_msg):
        self.camera_model.fromCameraInfo(camera_info_msg)
        img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_32FC1 = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        self.latest_depth_32FC1 = depth_32FC1.copy()

        # res = kinect_utils.filter_depth_noise(depth_32FC1)
        # depth_display = kinect_utils.normalize_depth_to_uint8(depth_32FC1.copy())
        # 
        # depth_32FC1[depth_32FC1 < 0.1] = np.finfo(np.float32).max

        cv2.imshow("Image window", img)
        # cv2.imshow("depth", depth_display)

        cv2.setMouseCallback("Image window", self.mouse_callback)

        cv2.waitKey(1)

        if self.pose:
            self.br.sendTransform(self.pose,(0,0,0,1),rospy.Time.now(),"clicked_object",self.camera_model.tfFrame())
            self.marker_pub.publish(self.generate_marker(rospy.Time(0), self.get_tf_frame(), self.pose))

    def get_current_raw_image(self):
        return self.bridge.imgmsg_to_cv2(self.latest_img_msg, "bgr8")

    def get_current_rect_image(self):
        output_img = np.ndarray(self.get_current_raw_image().shape)
        self.camera_model.rectifyImage(self.get_current_raw_image(), output_img)
        return output_img

    def get_tf_frame(self):
        return self.camera_model.tfFrame()

    def is_ready(self):
        return self.ready_

    def get_ray(self, uv_rect):
        return self.camera_model.projectPixelTo3dRay(uv_rect)

    def get_position_from_ray(self, ray, depth):
        """
        @brief      The 3D position of the object (in the camera frame) from a camera ray and depth value
        @param      ray    The ray (unit vector) from the camera centre point to the object point
        @param      depth  The norm (crow-flies) distance of the object from the camera
        @return     The 3D position of the object in the camera coordinate frame
        """

        # [ray_x * depth / ray_z, ray_y * depth / ray_z, ray_z * depth / ray_z]
        return [(i * depth) / ray[2] for i in ray]

    def generate_marker(self, stamp, frame_id, pose_3D):
        #marker_msg = Marker()
        #marker_msg.header.stamp = stamp
        #marker_msg.header.frame_id = frame_id
        #marker_msg.id = 0 #Marker unique ID

        ##ARROW:0, CUBE:1, SPHERE:2, CYLINDER:3, LINE_STRIP:4, LINE_LIST:5, CUBE_LIST:6, SPHERE_LIST:7, POINTS:8, TEXT_VIEW_FACING:9, MESH_RESOURCE:10, TRIANGLE_LIST:11
        #marker_msg.type = 2
        #marker_msg.lifetime = 1
        #marker_msg.pose.position = pose_3D
        marker_msg = Marker()
        marker_msg.header.frame_id = frame_id
        marker_msg.type = marker_msg.SPHERE
        marker_msg.action = marker_msg.ADD
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.pose.orientation.w = 1.0
        magicval_1 = 1.7
        marker_msg.pose.position.x = pose_3D[0]
        marker_msg.pose.position.y = pose_3D[1]
        marker_msg.pose.position.z = pose_3D[2]
        marker_msg.id = 1

        return marker_msg

    def process_ray(self, uv_rect, depth):
        ray = self.get_ray(uv_rect)
        pose = self.get_position_from_ray(ray,depth)

        print('Ray', ray, '\n')
        print('Pose', pose, '\n')

        return ray, pose

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # bolt_position_detector. detection_position()
            # clamp a number to be within a specified range
            clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

            #Small ROI around clicked point grows larger if no depth value found
            for bbox_width in range(20, int(self.latest_depth_32FC1.shape[0]/3), 5):
                tl_x = clamp(x-bbox_width/2, 0, self.latest_depth_32FC1.shape[0])
                br_x = clamp(x+bbox_width/2, 0, self.latest_depth_32FC1.shape[0])
                tl_y = clamp(y-bbox_width/2, 0, self.latest_depth_32FC1.shape[1])
                br_y = clamp(y+bbox_width/2, 0, self.latest_depth_32FC1.shape[1])
                print((x, y), (tl_x, tl_y, br_x, br_y))
                roi = self.latest_depth_32FC1[tl_y:br_y, tl_x:br_x]
                depth_distance = np.median(roi)

                if not np.isnan(depth_distance):
                    break

            print('distance (crowflies) from camera to point: {:.2f}m'.format(depth_distance))
            ray, self.pose = self.process_ray((x, y), depth_distance)
            testmotion.robot_position(ray[0],  ray[1] )

            
if __name__ == '__main__':
    try:
        rospy.init_node('depth_from_object', anonymous=True)
    
        while not rospy.is_shutdown():
            #camera = Camera('usb_cam', '/kinect2/qhd/image_color', '/kinect2/qhd/camera_info')
            #NEW
            camera = Camera('gloal_camera', '/gloal_camera/color/image_raw', '/gloal_camera/depth/image_raw', '/gloal_camera/color/camera_info')
            rospy.spin()

    except rospy.ROSInterruptException:
        print("Shutting down")
        cv2.destroyAllWindows()