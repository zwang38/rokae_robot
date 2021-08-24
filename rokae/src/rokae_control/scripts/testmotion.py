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


from time import sleep
from gazebo_msgs.srv import DeleteModel
import  bolt_position_detector 




def Delete_Part(item_name):
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        resp_delete = delete_model(item_name)
    except rospy.ServiceException as e:
        print("Delete Model service call failed: {0}".format(e))






def product_spawn():
    # This function spawn three types parts(screw1, screw2, woodbolt) in gazebo
    
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

    rospack = rospkg.RosPack()

    part_pkg = rospack.get_path('battery_pack_describe')
    
    print("please enter keyboard :  ho ,h ,v , t . the ros will load battery"  )

    print("{0} is hole_battery "  .format( 'ho'  ))
    print(".........................................................." )

    print("{0} is horizontal_battery " .format( 'h'  ))
    print(".........................................................."  )

    print("{0} is vertical_battery " .format( 'v'  ))
    print("..........................................................")


    print("{0} is tilt_battery ".format( 't'  ))
    print("..........................................................")


    battery_name=''
    input=raw_input()
    if input=='ho':
        with open(part_pkg+'/urdf/'+'hole_battery.urdf', "r") as hole_battery:
            product_xml = hole_battery.read()
            battery_name='hole_battery'
            hole_battery.close()
			
    elif input=='h':  
        with open(part_pkg+'/urdf/'+'h_battery.urdf', "r") as h_battery:
            product_xml = h_battery.read()
            battery_name='horizontal_battery'
            h_battery.close()     

    elif input=='v':  
        with open(part_pkg+'/urdf/'+'v_battery.xacro', "r") as v_battery:
            product_xml = v_battery.read()
            battery_name='vertical_battery'
            v_battery.close()   

    elif input=='t':  
        with open(part_pkg+'/urdf/'+'tilt_battery.urdf', "r") as tilt_battery:
            product_xml = tilt_battery.read()
            battery_name='tilt_battery'
            tilt_battery.close()   


    quat=tf.transformations.quaternion_from_euler(0,0,3.14) 
    orient = Quaternion(quat[0],quat[1],quat[2],quat[3])
    item_name   =   "{0}_product".format(battery_name)
    print("Spawning model:%s", item_name)
    item_pose   =   Pose(Point(x=0, y=0, z=0.12), orient)
    spawn_model(item_name, product_xml, "", item_pose, "table")
    rospy.sleep(0.5)
    return item_name



def part_pose_collect():
    parts_pose=[]
    model_pose = rospy.wait_for_message("gazebo/model_states",ModelStates)
    product_num = model_pose.name.index('vertical_battery_product')
    x = model_pose.pose[product_num].position.x
    y = model_pose.pose[product_num].position.y
    z = model_pose.pose[product_num].position.z
    X = model_pose.pose[product_num].orientation.x
    Y = model_pose.pose[product_num].orientation.y
    Z = model_pose.pose[product_num].orientation.z
    W = model_pose.pose[product_num].orientation.w
    euler=tf.transformations.euler_from_quaternion((X,Y,Z,W)) 
    parts_pose.append([x,y,z,euler[0],euler[1],euler[2]])     
    return parts_pose

def robot_move(group, x, y, z, R, P, Y):
    # moveit_commander.roscpp_initialize(sys.argv)
    # robot = moveit_commander.RobotCommander()

    group.clear_pose_targets()
    quat = tf.transformations.quaternion_from_euler(R, P, Y)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]
    group.set_pose_target(pose_target)

    plan = group.plan()
    group.execute(plan, wait=True)
    print(("test %f,%f,%f,%f,%f,%f")%(x,y,z,R,P,Y))



def robot_move_line(x1,y1,z1,x2,y2,z2):
    
    step = 5.0
    x_step_unit = (x2-x1)/step # target_pose - start pose / move_step
    y_step_unit= (y2-y1)/step # target_pose - start pose / move_step
    z_step_unit = (z2-z1)/step # target_pose - start pose / move_step

    waypoints = []
    robot_pose = geometry_msgs.msg.Pose()
    now_pose = group1.get_current_pose().pose

    # robot move following vector
    for i in range(1,int(step)):
        now_pose.position.x = x1 + x_step_unit*i
        now_pose.position.y = y1 + y_step_unit*i
        now_pose.position.z = z1 + z_step_unit*i
        waypoints.append(copy.deepcopy(now_pose))

    # last target pose add in memory
    robot_pose.position.x = x2
    robot_pose.position.y = y2
    robot_pose.position.z = z2
    robot_pose.orientation.x = now_pose.orientation.x
    robot_pose.orientation.y = now_pose.orientation.y
    robot_pose.orientation.z = now_pose.orientation.z
    robot_pose.orientation.w = now_pose.orientation.w
    waypoints.append(copy.deepcopy(robot_pose))

    plan, fraction = group1.compute_cartesian_path(waypoints,0.01,0.0) 
    group1.execute(plan,wait=True)



def goal_move_line(x,y,z): 
    step = 5.0
    waypoints = []
    robot_pose = geometry_msgs.msg.Pose()
    now_pose = group1.get_current_pose().pose
    x_step_unit = (x-now_pose.position.x)/step # target_pose - start pose / move_step
    y_step_unit= (y-now_pose.position.y)/step # target_pose - start pose / move_step
    z_step_unit = (z-now_pose.position.z)/step # target_pose - start pose / move_step
    robot_pose.orientation.x = now_pose.orientation.x
    robot_pose.orientation.y = now_pose.orientation.y
    robot_pose.orientation.z = now_pose.orientation.z
    robot_pose.orientation.w = now_pose.orientation.w

    # robot move following vector
    for i in range(1,int(step)):
        robot_pose.position.x = now_pose.position.x + x_step_unit*i
        robot_pose.position.y = now_pose.position.y + y_step_unit*i
        robot_pose.position.z = now_pose.position.z + z_step_unit*i
        waypoints.append(copy.deepcopy(robot_pose))

    # last target pose add in memory
    robot_pose.position.x = x
    robot_pose.position.y = y
    robot_pose.position.z = z
    waypoints.append(copy.deepcopy(robot_pose))

    plan, fraction = group1.compute_cartesian_path(waypoints,0.01,0.0) 
    group1.execute(plan,wait=True)


def robot_move_location(group, x, y, z, R, P, Y):
    robot_move(group, x, y, z, R, P, Y)

    
def robot_move_push(x1,y1,z1,x2,y2,z2):
    robot_move_line(x1,y1,z1,x2,y2,z2)


def robot_move_insert(x1,y1,z1,x2,y2,z2):
    robot_move_line(x1,y1,z1,x2,y2,z2)


def robot_position(x,y):
    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('motion_planning', anonymous=True)
    # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    # the robot:
    robot = moveit_commander.RobotCommander()
    # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    # to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    group_name1 = "arm"
    group1 = moveit_commander.MoveGroupCommander(group_name1)
    group1.set_planner_id("RRTConnectkConfigDefault")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    scene = moveit_commander.PlanningSceneInterface()
    # group1.set_named_target('home')
    rospy.sleep(2)

    # print('ray[0]0',ray[0])

    # 机器人移动到某个位置
    robot_move_location(group1,x,y,1.5, 3.14, 0, 0)
    rospy.sleep(5)




if __name__ == "__main__":
       
    # First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_planning', anonymous=True)

    # robot_position(0.38 , 0.27)


    # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    # the robot:
    robot = moveit_commander.RobotCommander()
    # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    # to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()


    print('add product,please input add')
    
    input_delete=raw_input()
    
    if input_delete=='add':
        load_battery_model=      product_spawn()
        print('load_battery_model',load_battery_model)


    group_name1 = "arm"
    group1 = moveit_commander.MoveGroupCommander(group_name1)
    group1.set_planner_id("RRTConnectkConfigDefault")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    scene = moveit_commander.PlanningSceneInterface()
    group1.set_named_target('home')
    rospy.sleep(2)

    # print('remove_battery_model,please input delete')
    
    # input_delete=raw_input()
    
    # if input_delete=='delete':
    #     Delete_Part(load_battery_model)
    
    parts_pose=part_pose_collect()
    print(parts_pose)

    # x,y=bolt_position_detector.detection_position()
    # print( "x={0},y={1}" .format(x,y))


    for model_num in range(len(parts_pose)):

        # 机器人移动到某个位置
        robot_move_location(group1,parts_pose[model_num][0],parts_pose[model_num][1],1.5, -3.14, 0, 0)
        rospy.sleep(5)
        # 下探
        robot_move_insert(parts_pose[model_num][0], parts_pose[model_num][1], 1.5, parts_pose[model_num][0], parts_pose[model_num][1], 1.3)
        # 推
        robot_move_push(parts_pose[model_num][0], parts_pose[model_num][1], 1.3, parts_pose[model_num][0], parts_pose[model_num][1], 1.2)
        # 回来，  好像有点问题，回不来的感觉
        # robot_move_line(parts_pose[model_num][0]+2, parts_pose[model_num][1]+2, 1.3 ,parts_pose[model_num][0], parts_pose[model_num][1], 1.3)
        # 回来，加套接
        robot_move_insert(parts_pose[model_num][0], parts_pose[model_num][1], 1.3, parts_pose[model_num][0], parts_pose[model_num][1], 1.2)
