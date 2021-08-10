#!/usr/bin/env python
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
        with open(part_pkg+'/urdf/'+'v_battery.urdf', "r") as v_battery:
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


rospy.init_node("sort_robot_program")
load_battery_model=      product_spawn()
print('load_battery_model',load_battery_model)

print('remove_battery_model,please input delete')

input_delete=raw_input()

if input_delete=='delete':
    Delete_Part(load_battery_model)
