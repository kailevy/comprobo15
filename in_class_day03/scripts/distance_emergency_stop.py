#!/usr/bin/env python

"""Node to drive Neato in a square (approximately) using odom measurements"""
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump

def move_forward():
    """Moves the robot forward until it bumps"""
    twister = Twist(linear=Vector3(x=0.5,y=0,z=0),angular=Vector3(x=0,y=0,z=0))
    pub.publish(twister)

def stop():
    """Stops the robot and marks the has_stopped flag as True"""
    twister = Twist(linear=Vector3(x=0,y=0,z=0),angular=Vector3(x=0,y=0,z=0))
    pub.publish(twister)
    global has_stopped
    has_stopped = True

def tooClose(scandata):
    threshold = .7
    if scandata[0] == 0:
        return False
    return scandata[0] <= threshold

def callback(msg):
    scandata = msg.ranges

    if not tooClose(scandata) and not has_stopped:
        move_forward()
    else:
        stop()

def listener():
    """Main function handler"""
    rospy.Subscriber('scan',LaserScan,callback,queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('drive_fwd')
    has_stopped = False

    while not rospy.is_shutdown():
        listener()
