#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from time import sleep

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.binary_image = None
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.Kp = -0.005


        cv2.namedWindow('video_window')
        cv2.namedWindow('binary_window')
        # cv2.namedWindow('threshold_image')
        # self.red_lower_bound = 0
        # cv2.createTrackbar('red lower bound', 'threshold_image', 0, 255, self.set_red_lower_bound)
        # cv2.setMouseCallback('video_window', self.process_mouse_event)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.binary_image = cv2.inRange(self.cv_image, (0,0,50), (20,20,255))

        #print self.cv_image.shape
        cv2.imshow('video_window', self.cv_image)
        cv2.imshow('binary_window', self.binary_image)
        cv2.waitKey(5)

    def set_red_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.red_lower_bound = val

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        image_info_window = 255*np.ones((500,500,3))
        cv2.putText(image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)

    def centerOfMass(self):
        moments = cv2.moments(self.binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']

    def run(self):
        """ The main run loop, processes images to determine and issue twist commands """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.centerOfMass()
            # determine center of mass of the image, stash in self.center_x and self.center_y

            imgWidth = self.cv_image.shape[0]
            offset = self.center_x - (.5 * imgWidth)
            # offset of the ball from the center of the image
            if  offset > (.25 * imgWidth) or offset < -(.25 * imgWidth):
                # the ball is way off, better stop to make sure we don't lose it
                 linVel = 0.0
            else:
                # go forward
                linVel = .3

            control = self.Kp * offset
            twist = Twist(linear=Vector3(x=linVel,y=0,z=0), angular=Vector3(z=control))
            self.pub.publish(twist)
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    sleep(1) # wait to acquire an image
    node.run()
