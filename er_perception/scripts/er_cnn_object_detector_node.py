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
import keras
import os
from er_perception.msg import ClassifiedImage, ClassifiedBoundingBox

DEBUG = True
PUBLISH = True

LABELS = ['Yellow Ball', 'Yellow Cube', 'Green Cube', 'Green Cylinder', 'Green Hollow Cube', 'Orange Cross', 'Patric', 'Red Cylinder', 'Red Hollow Cube', 'Red Ball', 'Blue Cube', 'Blue Triangle', 'Purple Cross', 'Purple Star', 'Other']

class simple_object_detector_node:

    def __init__(self):
        rospy.init_node('er_simple_object_detector_node', anonymous=True)

        self.box_publisher = rospy.Publisher("object_bounding_boxes", UInt16MultiArray, queue_size=1)
        self.box_class_publisher = rospy.Publisher("object_bounding_boxes_classified", ClassifiedImage, queue_size=1)

        script_folder = os.path.dirname(os.path.realpath(__file__))
	print(os.path.join(script_folder, 'cnn.h5'))
        self.cnn_model = keras.models.load_model(os.path.join(script_folder, 'cnn.h5'))
        self.cnn_model._make_predict_function()

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_rect_color', Image, self.detect_object, queue_size=1)

    def run(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def detect_object(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv2.balance_white(img)
            #img[:,:,0] = np.maximum(1.4*img[:,:,0], 0)
            #img[:,:,1] = np.minimum(1.3*img[:,:,1], 255)
            #img[:,:,2] = np.minimum(0.9*img[:,:,2], 255)
            # img[:,:,1] = np.maximum(1.3*img[:,:,1], 0)
            # img[:,:,:] = np.maximum(0.8*img[:,:,:], 0)
        except CvBridgeError as e:
            rospy.logerror('error when converting message to cv2 image')
            rospy.logerror(e)

        rectangles = find_bounding_boxes(img, False)


        # create message which will be sent
        # format will be a vector of length 4xn, where n is the number of detected bounding boxes
        # data is ordered like this: x1, y1, w1, h1, x2, y2, w2, h2, ..., xn, yn, wn, hn
        bounding_box_msg = UInt16MultiArray()
        classified_image_msg = ClassifiedImage()
        for box in rectangles:
            (x,y,w,h) = box
            roi = img[y:y+w, x:x+w]
            # roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            roi = np.asarray([cv2.resize(roi, (32, 32))], dtype=np.float32) / 255
            p = self.cnn_model.predict(roi)
            idx = np.argmax(p)

            if idx == 14:
                continue

            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, LABELS[idx] + '({0})'.format(p[0][idx]),(x, y + h/2), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)

            bounding_box_msg.data.append(x)
            bounding_box_msg.data.append(y)
            bounding_box_msg.data.append(w)
            bounding_box_msg.data.append(h)

            classified_bounding_box = ClassifiedBoundingBox()


            classified_bounding_box.x = x
            classified_bounding_box.y = y
            classified_bounding_box.w = w
            classified_bounding_box.h = h
            classified_bounding_box.class_id = idx

            classified_image_msg.bounding_boxes.append(classified_bounding_box)

        classified_image_msg.image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        classified_image_msg.image.header = data.header
        if DEBUG:
            cv2.imshow('image', img)
            cv2.waitKey(1)

        if PUBLISH:
            self.box_publisher.publish(bounding_box_msg)
            self.box_class_publisher.publish(classified_image_msg)

        rospy.logdebug('published the following bounding boxes: ' + str(bounding_box_msg))

if __name__ == '__main__':
    li = simple_object_detector_node()
    li.run()
