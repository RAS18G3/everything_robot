#!/usr/bin/env python
#import sys
import roslib
import rospy
import cv2
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from simple_object_detector import find_bounding_boxes

DEBUG = True

class simple_object_detector_node:

    def __init__(self):
        rospy.init_node('er_simple_object_detector_node', anonymous=True)

        self.box_publisher = rospy.Publisher("object_bounding_boxes", UInt16MultiArray, queue_size=1)

        # Needed to convert Image message to cv2 image type
        self.bridge = CvBridge()

        # Subscriber listening for the images sent by the camera, and set the callback function
        self.image_sub = rospy.Subscriber('camera/rgb/image_rect_color', Image, self.detect_object, queue_size=1)

    def run(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def detect_object(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerror('error when converting message to cv2 image')
            rospy.logerror(e)

        rectangles = find_bounding_boxes(img, DEBUG)

        # create message which will be sent
        # format will be a vector of length 4xn, where n is the number of detected bounding boxes
        # data is ordered like this: x1, y1, w1, h1, x2, y2, w2, h2, ..., xn, yn, wn, hn
        bounding_box_msg = UInt16MultiArray()
        for box in rectangles:
            (x,y,w,h) = box
            bounding_box_msg.data.append(x)
            bounding_box_msg.data.append(y)
            bounding_box_msg.data.append(w)
            bounding_box_msg.data.append(h)

        self.box_publisher.publish(bounding_box_msg)

        rospy.logdebug('published the following bounding boxes: ' + str(bounding_box_msg))

if __name__ == '__main__':
    li = simple_object_detector_node()
    li.run()
