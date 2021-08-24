#!/usr/bin/python
# -*- coding: utf-8 -*-

# tf2_ros_example.py: example showing how to use tf2_ros API
# Author: Ravi Joshi
# Date: 2018/12/6

# import modules
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped


def transform_point(transformation, point_wrt_source):
    point_wrt_target = \
        tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source),
            transformation).point
    return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]


def get_transformation(source_frame, target_frame,
                       tf_cache_duration=2.0):
    tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
    tf2_ros.TransformListener(tf_buffer)

    # get the tf at first available time
    try:
        transformation = tf_buffer.lookup_transform(target_frame,
                source_frame, rospy.Time(0), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s'
                     % source_frame, target_frame)
    return transformation


def main():
    # initialize ros node
    rospy.init_node('tf2_ros_example', anonymous=True)

    # define source and target frame
    source_frame = 'table'
    target_frame = 'rokae_link7'

    # define a source point
    point_wrt_source = Point(0.1, 0.2,1.3)

    transformation = get_transformation(source_frame, target_frame)
    point_wrt_target = transform_point(transformation, point_wrt_source)
    print point_wrt_target


if __name__ == '__main__':
    main()