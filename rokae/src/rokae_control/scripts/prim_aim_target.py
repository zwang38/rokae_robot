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


# from PIL import Image,ImageDraw
# import numpy as np 


class PrimAimTarget:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.action_params = ['rgb_img', 'depth_img', 'camera_model', 'timestamp']

    def action(self, all_info):
        for param in self.action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified")
        return True


