#!/usr/bin/env python
import roslib
roslib.load_manifest('cv')
#import sys
import rospy
import cv2
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
from pynput import keyboard

'''
Do not forget to source worskpace every time opening a new terminal
source ./delev/setup.bash
Do not forget to make python file executable everytime creating a new pyhon node
chmod +x file_name.py
'''


class image_recorder(object):
    """docstring for listener"""

    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('listener', anonymous=True)
        # set the csllback function for the camera topic
        rospy.Subscriber('camera/rgb/image_rect_color', Image, self.callback)

        self.camera_rolling = False
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.writer = None
        self.img_nr = 0

        self.bridge = CvBridge()

        with keyboard.Listener(on_press=lambda key: self.on_press(key),
                               on_release=lambda key: self.on_release(key)) as listener:
            listener.join()

    def on_press(self, key):
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
            if key.char == 'r':
                self.camera_rolling = True
            elif key.char == 'e':
                self.camera_rolling = False
            elif key.char == 'q' or key == keyboard.Key.esc:
                exit()

        except AttributeError:
            pass

    def on_release(self, key):
        try:
            pass

        except AttributeError:
            pass



    def callback(self, data):

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('wrong when converting message to cv2 image')
            print(e)

        if self.camera_rolling:
            # make sure there is a img folder where the node is run from
            print('storing img' + str(self.img_nr))
            cv2.imwrite('img/image_' + str(self.img_nr) + '.jpeg', frame)
            self.img_nr += 1
            self.camera_rolling = False

        cv2.imshow('frame', frame)
        cv2.waitKey(1)

    def run(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo('I have started!!!')
        rospy.spin()

if __name__ == '__main__':
    try:
        li = image_recorder()
        li.run()
    except rospy.ROSInterruptException:
        pass
