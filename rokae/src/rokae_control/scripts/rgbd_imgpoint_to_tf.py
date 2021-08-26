#!/usr/bin/env python
#-*- coding: UTF-8 -*-
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
import templateMatching
# from PIL import Image,ImageDraw
# import numpy as np 

class Camera():
    def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic,  arm_camera):

        self.camera_name = camera_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.arm_camera=arm_camera

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
        self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_camera_info], queue_size=30, slop=0.2)
        #self.tss = message_filters.TimeSynchronizer([sub_rgb], 10)

        self.tss.registerCallback(self.callback)


    def  detection_position(self,image):
        x_position=0
        y_position=0
        gray_img= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        # cv2.imshow('gray_img',gray_img)
        # cv2.waitKey()

        img = cv2.medianBlur(gray_img,5)
        circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=35,minRadius=0,maxRadius=0)
        if circles is None:
            return img.shape[0]/2,img.shape[1]/2
        else :

            circles = np.uint16(np.around(circles))
            
    
            # print('image is ok ...')
            for i in circles[0,:]:
                print ('这是第{0}个圆心'.format(i+1))
                #画出来圆的边界
                cv2.circle(image,(i[0],i[1]),i[2],(0,0,255),2)
                #画出来圆心
                cv2.circle(image,(i[0],i[1]),2,(0,255,255),3)
                x_position=i[0]
                y_position=i[1]
            # print('image is no ok ...')
            print( "圆心 x={0},y={1}" .format(x_position, y_position))
            cv2.imwrite('src/rokae_control/images/rgb_img.jpg', image)

            # cv2.imshow("Circle",image)
            # cv2.waitKey()
            # cv2.destroyAllWindows()
            return x_position,y_position


    def cal_pos(self,img_clr):
        try:
            img = cv2.cvtColor(img_clr,cv2.COLOR_BGR2GRAY)
            # resize image
            # cv2.imshow('gray_img',img)
            # cv2.waitKey(1)

            print(img.shape)
            scale = 800.0 / img.shape[1]
            resized = cv2.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale)))
            img_clr = cv2.resize(img, (int(img_clr.shape[1] * scale), int(img_clr.shape[0] * scale)))
            
            # Threshold
            kernel = np.ones((1, 3), np.uint8)
            im = cv2.morphologyEx(resized, cv2.MORPH_BLACKHAT, kernel, anchor=(1, 0))
            thresh, im = cv2.threshold(resized, 140, 255, cv2.THRESH_BINARY)
            
            # dilation and erosion
            kernel = np.ones((1, 3), np.uint8)
            im = cv2.morphologyEx(im, cv2.MORPH_DILATE, kernel, anchor=(2, 0), iterations=2) 
            im = cv2.morphologyEx(im, cv2.MORPH_CLOSE, kernel, anchor=(2, 0), iterations=2)  
            
            # Remove elements that are too small to fit/excess noise removal
            kernel = np.ones((3, 3), np.uint8)
            im = cv2.morphologyEx(im, cv2.MORPH_OPEN, kernel, iterations=1)
            
            # contour detection
            # # im2, contours, hierarchy = cv2.findContours(im,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            # cnts = cv2.findContours(im.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # cnts = imutils.grab_contours(cnts)
            
            contours = cv2.findContours(im,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            unscale = 1.0 / scale
            try:
                contours = sorted(contours, key=cv2.contourArea)[:]
                c=0
                rect_c = 0
                if contours != None:
                    for contour in contours:
                        c = c+1
                        peri = cv2.arcLength(contour, True)
                        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                        # segment based on area
                        print(str(c)+ " Contour area: ", cv2.contourArea(contour))
                        if (cv2.contourArea(contour) >=  401135  or cv2.contourArea(contour) <= 20825 ):
                            rect_c = rect_c+1
                            print("Missed : "+str(rect_c)+" Area of missed: "+ str(cv2.contourArea(contour)))
                            continue
                        #draw rect of smallest possible size for contour    
                        rect = cv2.minAreaRect(contour)
                        #print("Detected Contour area: ", cv2.minAreaRect(contour))
                        #rect = ((int(rect[0][0] * unscale), int(rect[0][1] * unscale)),(int(rect[1][0] * unscale), int(rect[1][1] * unscale)),rect[2])
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        cv2.drawContours(img_clr,[box],0,(0,0,255),thickness=2)
                
                        (x,y),radius = cv2.minEnclosingCircle(contour)
                
                        print('x',int(x))
                        print('y',int(y))
                        print('radius',int(radius))
                
                return int(x),int(y)
            except rospy.ROSInterruptException:
                print("Shutting down")
                cv2.destroyAllWindows()
        except rospy.ROSInterruptException:
            print("Shutting down")
            cv2.destroyAllWindows()

    def templateMatching(self,img_path):

        template_path='src/battery_pack_describe/bolt.jpg'

        small_image = Image.open(template_path)
        big_image =Image.open(img_path)
    
        big_image_rgb = big_image
        small_image = small_image.convert('L')
    
        big_image = big_image.convert('L')
        
        big = np.array(big_image)
        small = np.array(small_image)
        
        rand_point = []
        for i in range(50):
            rand_point.append((np.random.randint(0,small.shape[0]-1) ,np.random.randint(0,small.shape[1]-1)))
        
        R = np.zeros([big.shape[0]-small.shape[0],big.shape[1]-small.shape[1]])
        
        sMean = np.mean(small)
        for i in range(R.shape[0]):
            for j in range(R.shape[1]):
                loss = 0 
                mean = np.mean(big[i:i+small.shape[0],j:j+small.shape[1]]) 
                for n in range(len(rand_point)):
                    point = rand_point[n]
        
                    loss += np.abs( big[i+point[0],j+point[1]] - mean - small[point[0],point[1]] + sMean)
                    if loss >= 150 or n==len(rand_point)-1: 
                        R[i,j] = nself
                        break 
        index = np.unravel_index(R.argmax(), R.shape)
         
        xy = [(index[1],index[0]),(index[1]+small.shape[1],index[0]+small.shape[0])]
        print('index[1]',index[1])
        print('index[0]',index[0])
        print('c[1]',index[1]+small.shape[1]/2)
        print('c[0]',index[0]+small.shape[0]/2)
    
    
        big_image = big_image_rgb
        draw = ImageDraw.Draw(big_image)
        # draw.rectangle(xy,outline='red',width=1)
        draw.rectangle(xy, fill ="#ffff33", outline ="red")
        big_image.show()
        # big_image.save("output.jpg")
        return index[1]+small.shape[1]/2,index[0]+small.shape[0]/2



    def callback(self, rgb_msg, depth_msg, camera_info_msg):
        try:
            rgb_img_path='src/rokae_control/images/rgb_img.jpg'

            self.camera_model.fromCameraInfo(camera_info_msg)
            img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_32FC1 = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
            self.latest_depth_32FC1 = depth_32FC1.copy()

            rospy.loginfo('receiving image')
            cv2.imwrite(rgb_img_path, img)
            #模板匹配

            x_templateMatching,y_templateMatching=templateMatching.main( rgb_img_path )
            x_circle,y_circle=self.detection_position(img)
            x=x_templateMatching
            y=y_templateMatching

            # cv2.imshow("img", img)
            # cv2.waitKey()


            # if  self.arm_camera   :
            #     x,y=self.detection_position(img)
            # else:
            #     x,y= self.cal_pos(img)

   
            # x,y= self.cal_pos(img)

            self.mouse_callback(x,y,img)
            # cv2.setMouseCallback("Image window", self.mouse_callback(img))
            # cv2.waitKey(1)
    
            if self.pose:
                self.br.sendTransform(self.pose,(0,0,0,1),rospy.Time.now(),"clicked_object",self.camera_model.tfFrame())
                self.marker_pub.publish(self.generate_marker(rospy.Time(0), self.get_tf_frame(), self.pose))
        except rospy.ROSInterruptException:
            print("Shutting down")
            cv2.destroyAllWindows()



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

    # def mouse_callback(self, event, x, y, flags, param):

    def mouse_callback(self,x,y,img):
        # if event == cv2.EVENT_LBUTTONDOWN:

        try:
            # bolt_position_detector.detection_position(img)
            # x,y=self.detection_position(img)
            #模板匹配
            # x,y=templateMatching.main(img)

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

            print('distance (crowflies) from camera to point: {:.2f}mm'.format(depth_distance))
            ray, self.pose = self.process_ray((x, y), depth_distance)
            print( 'image x', x )
            print( 'image y', y )
            print( 'ray x',ray[0] )
            print( 'ray y', ray[1] )

            testmotion.robot_position(ray[0],  ray[1]  ,depth_distance/1000 +0.8)
            # testmotion.robot_position(0.4,0,1.5)
        except rospy.ROSInterruptException:
            print("Shutting down")
            cv2.destroyAllWindows()
            
if __name__ == '__main__':
    testmotion.robot_position(0,0,1.5)

    try:
        rospy.init_node('depth_from_object')
        

        while not rospy.is_shutdown():
            #camera = Camera('usb_cam', '/kinect2/qhd/image_color', '/kinect2/qhd/camera_info')
            # print('arm camera or gloal camera, if arm please input arm,enter "enter" ')
            # input_delete=raw_input()
    
            # if input_delete=='arm':
            #     arm_camera=True
                 # arm_camera
            arm_camera=True
            camera = Camera('camera', '/camera/color/image_raw', '/camera/depth/image_raw', '/camera/color/camera_info',arm_camera)
            # else :
            #     # gloal camera 
            #     arm_camera=False
            #     camera = Camera('gloal_camera', '/gloal_camera/color/image_raw', '/gloal_camera/depth/image_raw', '/gloal_camera/color/camera_info',arm_camera)
            
            
            # camera = Camera('gloal_camera', '/gloal_camera/color/image_raw', '/gloal_camera/depth/image_raw', '/gloal_camera/color/camera_info',arm_camera)

            rospy.spin()

    except rospy.ROSInterruptException:
        print("Shutting down")
        cv2.destroyAllWindows()