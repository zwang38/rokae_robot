# #!/usr/bin/env python
# #!coding=utf-8
 
# import rospy
# import numpy as np
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
 
# gloal_cam_path  = 'src/gloal_image_file'    # 已经建立好的存储cam0 文件的目录
# arm_cam_path  ='src/gloal_image_file'  
 
# def callback(data):
#     # define picture to_down' coefficient of ratio
#     scaling_factor = 0.5
#     global count,bridge
#     count = count + 1
#     if count == 1:
#         count = 0
#         cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
#         timestr = "%.6f" %  data.header.stamp.to_sec()
#               #%.6f表示小数点后带有6位，可根据精确度需要修改；
#         image_name = timestr+ ".jpg" #图像命名：时间戳.jpg
#         cv2.imwrite(cam0_path + image_name, cv_img)  #保存；
#         cv2.imshow("frame" , cv_img)
#         cv2.waitKey(3)
#     else:
#         pass
 
# def displayWebcam():
#     rospy.init_node('webcam_display', anonymous=True)
#     # make a video_object and init the video object
#     global count,bridge
#     count = 0
#     bridge = CvBridge()
#     rospy.Subscriber('~/rokae/table/gloal_cameracolor/image', Image, callback)
#     rospy.spin()
 
# if __name__ == '__main__':
#     displayWebcam()



#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('src/gloal_image_file/camera_image.jpeg', cv2_img)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/gloal_camera/color/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()