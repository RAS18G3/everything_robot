#!/usr/bin/env python
#import sys
import roslib
import rospy
import cv2
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

##### Global variables ##################
SATURATION_THRESHOLD = 180 # decrease if red/orange/green/blue/yellow are not recognized
VIOLET_HUE_LOW = 115 # increase the violet range if violet is not recognized
VIOLET_HUE_HIGH = 145
VIOLET_SATURATION = 40 # decrease if violet is not recognized
W_MIN = 35
H_MIN = 35
# do the opposite if there are too many false positives

DEBUG = False

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

        # Apply smoothing to the image
        # could be worth it to resize, but bounding boxes at the bottom would have to be adjusted!
        # img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_CUBIC)

        # blur the image to remove potential noise
        img = cv2.blur(img, (5, 5))

        # convert to HSV space for detection
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 5 masks are produced
        # 1: threshold only the saturation value -> good for orange, red, yellow, green, blue
        ret, thresh1 = cv2.threshold(hsv[:, :, 1], SATURATION_THRESHOLD, 255, cv2.THRESH_BINARY)
        # 2: threshold the hue for violet
        thresh2 = cv2.inRange(hsv[:, :, 0], VIOLET_HUE_LOW, VIOLET_HUE_HIGH)
        # 3: to make violet more reliable have a second, big range saturation threshold
        ret, thresh3 = cv2.threshold(hsv[:, :, 1], VIOLET_SATURATION, 255, cv2.THRESH_BINARY)
        # 4: and combine mask 2 and 3 -> this will be only violet objects
        thresh4 = cv2.bitwise_and(thresh2, thresh3)
        # 5: or combine 1 and 4 to get all the objects
        thresh5 = cv2.bitwise_or(thresh1, thresh4)

        # first dilate and then erode, this will merge bounding boxes which are close
        thresh5 = cv2.dilate(thresh5, cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20)), iterations=3)
        thresh5 = cv2.erode(thresh5, cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10)), iterations=2)

        if DEBUG:
            # show the different masks, this can be helpful if the thresolds must be tuned
            cv2.imshow('mask 1', thresh1)
            cv2.imshow('mask 2', thresh2)
            cv2.imshow('mask 3', thresh3)
            cv2.imshow('mask 4', thresh4)
            cv2.imshow('mask 5', thresh5)

        # find contour of the final mask, as this can easily be used to find the bounding boxes
        _, contours, hierarchy = cv2.findContours(thresh5, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

        if DEBUG:
            rospy.logdebug(contours)

        # find the bounding boxes
        rectangles = []
        for contour in contours:
            rectangles.append(cv2.boundingRect(contour))


        if DEBUG:
            # draw the bounding boxes and contours on the image
            #cv2.drawContours(img, contours, -1, (255, 255, 0), 3)
            for rectangle in rectangles:
                (x,y,w,h) = rectangle
                if w >= W_MIN and  h >= H_MIN:
                    cv2.rectangle(img, (rectangle[0], rectangle[1]), (rectangle[0] + rectangle[2], rectangle[1] + rectangle[3]),
                    (0, 255, 0), 2)
            cv2.imshow('image', img)
            cv2.waitKey(1)


        # create message which will be sent
        # format will be a vector of length 4xn, where n is the number of detected bounding boxes
        # data is ordered like this: x1, y1, w1, h1, x2, y2, w2, h2, ..., xn, yn, wn, hn
        bounding_box_msg = UInt16MultiArray()
        for box in rectangles:
            (x,y,w,h) = box
            if w >= W_MIN and  h >= H_MIN:
                bounding_box_msg.data.append(x)
                bounding_box_msg.data.append(y)
                bounding_box_msg.data.append(w)
                bounding_box_msg.data.append(h)

        self.box_publisher.publish(bounding_box_msg)

        rospy.logdebug('published the following bounding boxes: ' + str(bounding_box_msg))

if __name__ == '__main__':
    li = simple_object_detector_node()
    li.run()
