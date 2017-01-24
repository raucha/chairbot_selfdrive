#!/usr/bin/env python
# -*- coding:utf-8 -*-

# """
# /map->/base_footprintの距離を
# /mimamorukun/dbposeで発行する．
# この値はB-Senのネットワークに送信されてTMS_DBに保存される．
# """

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose



def trans2pose(trans):
    pose = Pose()
    pose.position.x = trans.transform.translation.x
    pose.position.y = trans.transform.translation.y
    pose.position.z = trans.transform.translation.z
    pose.orientation = trans.transform.rotation
    return pose


if __name__ == "__main__":
    rospy.init_node('pub_dbpose')
    rospy.loginfo("hello world")
    dbpose_pub = rospy.Publisher('/mimamorukun/dbpose', Pose, queue_size=10)
    tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    listener = tf2_ros.TransformListener(tfBuffer)
    # global tfBuffer
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
        now = rospy.get_rostime()
        if not tfBuffer.can_transform("map", "base_footprint", now,timeout=rospy.Duration(3.0)):
              rospy.logwarn("can_transform() timeout")
              continue
        trans = tfBuffer.lookup_transform("map", "base_footprint", now, timeout=rospy.Duration(1.0))
        dbpose = trans2pose(trans)
        dbpose_pub.publish(dbpose)

