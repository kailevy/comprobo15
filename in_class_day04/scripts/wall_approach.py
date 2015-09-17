#!/usr/bin/env python
"""Ros node to approach wall with proporational control"""

import rospy
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from geometry_msgs.msg import Twist, Vector3

class WallApproach(object):
    def __init__(self, desired_distance):
        rospy.init_node('wall_approach')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('/bump', Bump, self.process_bump)
        self.desired_distance = desired_distance
        self.speed_constant = 0.5
        self.threshold = 0.01
        self.wall_distance = 0
        self.bumped = False
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def process_scan(self, msg):
        self.wall_distance = msg.ranges[0]

    def process_bump(self, msg):
        pass

    def move_to_wall(self):
        off_distance = (self.wall_distance-self.desired_distance)
        velocity = self.speed_constant * off_distance
        twist = Twist(linear=Vector3(x=velocity))
        self.pub.publish(twist)

    def twist(self):
        self.pub.publish(Twist(angular=Vector3(z=0.5)))

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.wall_distance >= self.threshold:
                self.move_to_wall()
            else:
                self.twist()
            r.sleep()

if __name__ == '__main__':
    robot = WallApproach(2)
    robot.run()
