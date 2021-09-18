#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, random, copy
import rospy, tf, rospkg
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
import  testmotion

import select, termios, tty

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
import image_geometry
import os



global capture_number


def load_obstacle():

    print('加载障碍物,please input add')
    input_delete=raw_input()

    if input_delete=='add':
        concept_demo.product_spawn()


class Camera():

    def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic):

        self.camera_name = camera_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic

        self.pose = None


        self.br = tf.TransformBroadcaster()

        # Have we recieved camera_info and image yet?
        self.ready_ = False

        self.bridge = CvBridge()

        self.camera_model = image_geometry.PinholeCameraModel()
        print(
            'Camera {} initialised, {}, , {}'.format(self.camera_name, rgb_topic, depth_topic, camera_info_topic))
        print('')

        q = 1
        self.sub_rgb = message_filters.Subscriber(rgb_topic, Image, queue_size=q)
        self.sub_depth = message_filters.Subscriber(depth_topic, Image, queue_size=q)
        self.sub_camera_info = message_filters.Subscriber(camera_info_topic, CameraInfo, queue_size=q)
        # self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_camera_info], queue_size=15, slop=0.4)
        self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_camera_info],
                                                               queue_size=30, slop=0.2)
        # self.tss = message_filters.TimeSynchronizer([sub_rgb], 10)

        self.tss.registerCallback(self.callback)
        self.capture = False
        directory = './images'
        if not os.path.exists(directory):
            os.makedirs(directory)

    def callback(self, rgb_msg, depth_msg, camera_info_msg):
        if not self.capture:
            return
        rgb_img_path = './images/rgb_img_%s.jpg'
        depth_img_path = './images/depth_img_%s.png'

        self.camera_model.fromCameraInfo(camera_info_msg)
        img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        print('receiving image')
        time_str = rospy.get_time()
        cv2.imwrite(rgb_img_path%(time_str), img)
        cv2.imwrite(depth_img_path%(time_str), depth_img)
        self.capture = False

    def set_capture(self):
        self.capture = True


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
    q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)
    print( '%s: position (%.2f %.2f %.2f) orientation (%.2f %.2f %.2f %.2f) RPY (%.2f %.2f %.2f)' % \
        (effector, pose.position.x, pose.position.y, pose.position.z, \
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, \
        rpy[0], rpy[1], rpy[2]))



def set_align_vertical_capture(ee_pose ,   x_bolt, y_bolt, z_bolt)   :
    x_battery  ,  y_battery= get_gazebo_model_pose()
    # 相机旋转
    delta_rpy_random=random.randint(-314,314)
    delta_rpy_random=float(delta_rpy_random)/float(100.0)

    q = (ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)

    # Z轴移动范围
    z_hight=random.randint(118,150)
    z_random_hight=float(z_hight)/float(100.0)

    ee_pose.position.x =x_battery +x_bolt
    ee_pose.position.y =y_battery +y_bolt

    print ' range z  '
    ee_pose.position.z = z_random_hight
    # ee_pose.position.z = 1.18

    #rpy:变换
    q = tf.transformations.quaternion_from_euler(-math.pi, rpy[1], rpy[2]+delta_rpy_random)
    ee_pose.orientation.x = q[0]
    ee_pose.orientation.y = q[1]
    ee_pose.orientation.z = q[2]
    ee_pose.orientation.w = q[3]
    # set_arm_pose(group, ee_pose, effector)
    if  set_arm_pose(group, ee_pose, effector):
        camera.set_capture()
    else :
        ee_pose = group.get_current_pose(effector).pose
    print_pose(ee_pose)




def set_move_vertical_capture(ee_pose ,   x_bolt, y_bolt, z_bolt)   :
    x_battery  ,  y_battery= get_gazebo_model_pose()

    delta_rpy_random=random.randint(-314,314)
    delta_rpy_random=float(delta_rpy_random)/float(100.0)
    q = (ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)

    x_center_bolt_pos=x_battery+x_bolt
    y_center_bolt_pos=y_battery+y_bolt
    # ee_pose.position.x =x_battery +x_bolt
    # ee_pose.position.y =y_battery +y_bolt

    x_delta=random.randint(-25,25)
    x_delta_random=float(x_delta)/float(1000.0)

    y_delta=random.randint(-25,25)
    y_delta_random=float(y_delta)/float(1000.0)


    z_hight=random.randint(118,150)
    z_random_hight=float(z_hight)/float(100.0)

    print(x_delta)
    print(y_delta_random)
    # now_pose = group.get_current_pose().pose

    # xyz:变换
    # ee_pose.position.x += x_delta_random
    # if  ee_pose.position.x < -0.25  or  ee_pose.position.x >0.25 :
    ee_pose.position.x =x_delta_random +x_center_bolt_pos

    # ee_pose.position.y += y_delta_random
    # if  ee_pose.position.y < -0.35  or  ee_pose.position.y >0.35 :
    ee_pose.position.y =y_delta_random+y_center_bolt_pos

    # ee_pose.position.z += z_random_hight
    # if  ee_pose.position.z< 1.18  or  ee_pose.position.z>1.50:
    ee_pose.position.z=z_random_hight

    #rpy:变换
    q = tf.transformations.quaternion_from_euler(-math.pi, rpy[1], rpy[2]+delta_rpy_random)
    ee_pose.orientation.x = q[0]
    ee_pose.orientation.y = q[1]
    ee_pose.orientation.z = q[2]
    ee_pose.orientation.w = q[3]

    # set_arm_pose(group, ee_pose, effector)

    if  set_arm_pose(group, ee_pose, effector):
        camera.set_capture()
        # 本想定义个采集次数，全局变量不给编译
        # capture_number= capture_number+1
        # if capture_number is 1000:
        #     print('采集了1000次{0}'.format(capture_number))
    else :
        ee_pose = group.get_current_pose(effector).pose

    print_pose(ee_pose)

def set_align_tilt_capture(ee_pose):
    print( 'location 45度到倾斜角，采集图像')
    tf_angle=-math.pi+math.pi/4

    z_hight=random.randint(0,5)
    z_random_hight=float(z_hight)/float(100.0)
    y_delta_random=z_random_hight

    q = tf.transformations.quaternion_from_euler(tf_angle,0,0)
    ee_pose.position.x=-0.060
    ee_pose.position.y=-0.233-y_delta_random
    ee_pose.position.z=1.131+z_hight

    ee_pose.orientation.x = q[0]
    ee_pose.orientation.y = q[1]
    ee_pose.orientation.z = q[2]
    ee_pose.orientation.w = q[3]

    if  set_arm_pose(group, ee_pose, effector):
        camera.set_capture()
    else :
        ee_pose = group.get_current_pose(effector).pose

    print_pose(ee_pose)



def set_move_tilt_capture(ee_pose):

    print( 'location 45度到倾斜角，采集图像')
    tf_angle=-math.pi+math.pi/4

    delta_rpy_random=random.randint(-314,314)
    delta_rpy_random=float(delta_rpy_random)/float(100.0)

    q = (ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)


    # randint中的数值为极限值
    x_delta=random.randint(-110,25)
    x_delta_random=float(x_delta)/float(1000.0)

    y_delta=random.randint(-430,-250)
    y_delta_random=float(y_delta)/float(1000.0)

    z_hight=random.randint(115,120)
    z_random_hight=float(z_hight)/float(100.0)

    print(x_delta)
    print(y_delta_random)
    # now_pose = group.get_current_pose().pose

    ee_pose.position.x =x_delta_random
    ee_pose.position.y=y_delta_random
    ee_pose.position.z=z_random_hight
    # ee_pose.position.x =-0.11
    # ee_pose.position.y=-0.25
    # ee_pose.position.z=1.19


    #rpy:变换
    q = tf.transformations.quaternion_from_euler(tf_angle, rpy[1], rpy[2])
    ee_pose.orientation.x = q[0]
    ee_pose.orientation.y = q[1]
    ee_pose.orientation.z = q[2]
    ee_pose.orientation.w = q[3]

    # set_arm_pose(group, ee_pose, effector)

    if  set_arm_pose(group, ee_pose, effector):
        camera.set_capture()
    else :
        ee_pose = group.get_current_pose(effector).pose

    print_pose(ee_pose)



def get_gazebo_model_pose():
    parts_pose=[]
    model_pose = rospy.wait_for_message("gazebo/model_states",ModelStates)
    # for count in range(len(model_pose.name)-1):
    current_product=len(model_pose.name)-1
    name = model_pose.name[current_product]
    x = model_pose.pose[current_product].position.x
    y = model_pose.pose[current_product].position.y

    # ee_pose = model_pose.pose[current_product].pose


    # parts_pose.append([name, x, y])
    
    return   x ,  y



if __name__=="__main__":

    x_bolt=-0.057323   # 数值大，向下
    y_bolt=0.03838  # 数值大，向左
    z_bolt=1.1815

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
    camera = Camera('camera', '/camera/color/image_raw', '/camera/depth/image_raw',
                    '/camera/color/camera_info')
    # testmotion.robot_position(-0.0595,0.0419,1.18)   #移动到电池包

    # set_align_tilt_capture(ee_pose)

    # x_bolt=-0.057323   # 数值大，向下
    # y_bolt=0.03838  # 数值大，向左
    # z_bolt=1.1815

    # x_bolt=-0.058   # 数值大，向下
    # y_bolt=0.0417 # 数值大，向左
    # z_bolt=1.1815


    x_battery  ,  y_battery= get_gazebo_model_pose()
    testmotion.robot_position(x_battery + x_bolt , y_battery + y_bolt,z_bolt)   #移动到电池包

    rospy.sleep(2)

    #加载障碍物
    print('请输入：add,加载障碍物,不加载直接回车')
    input=raw_input()
    if input=='add':
        load_obstacle()


    print('请输入：va,进行垂直方向对齐采集；垂直方向非对齐采集，请输入vn')
    input=raw_input()
    if input=='va':
        for i in  range(100):  #采样100次
            print('current {0}'.format(i))
            set_align_vertical_capture(ee_pose,  x_bolt, y_bolt, z_bolt)
            # if i%100==0:
            #     testmotion.robot_position(x_bolt,y_bolt,z_bolt)   #移动到电池包

    elif input=='vn':
        for i in  range(2000):  #采样300次
            print('current {0}'.format(i))
            set_move_vertical_capture(ee_pose,  x_bolt, y_bolt, z_bolt)



    print('请输入：ta,进行垂直方向进行对齐.倾斜采集，请输入tn')
    input=raw_input()
    if input=='ta':
        for i in  range(100):
            print('current {0}'.format(i))
            set_align_tilt_capture(ee_pose)
    elif input=='tn':
        for i in  range(300):  #采样300次
            set_move_tilt_capture(ee_pose)


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)









    # robot_move_circle(parts_pose[model_num][0], parts_pose[model_num][1], 1.5)

    # robot_move_rectangle(parts_pose[model_num][0], parts_pose[model_num][1], 1.5)

    # robot_move_insert(parts_pose[model_num][0], parts_pose[model_num][1], 1.5, parts_pose[model_num][0], parts_pose[model_num][1], 1.3)

    # robot_move_insert(parts_pose[model_num][0], parts_pose[model_num][1], 1.3, parts_pose[model_num][0], parts_pose[model_num][1], 1.2)
