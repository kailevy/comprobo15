#!/usr/bin/env python

"""Script used to explore ROS messages with python"""

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import rospy
rospy.init_node('test_message')

point_msg = Point(z=1.0, x=2.0)
header_msg = Header(stamp=rospy.Time.now(), frame_id="odom")

msg = PointStamped(header=header_msg, point=point_msg)

pub = rospy.Publisher("/my_point", PointStamped, queue_size=10)

r = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(msg)
    r.sleep()
