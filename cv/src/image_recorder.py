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

        rospy.Subscriber("camera_start", Bool, self.start_camera)
        rospy.Subscriber("stop_camera", Bool, self.stop_camera)

        self.camera_rolling = True
        self.camera_stop = False
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.writer = None
        self.counter = 0

        self.bridge = CvBridge()

    def run(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo('I have started!!!')
        rospy.spin()

    def start_camera(self, data):
        self.camera_rolling = data.data

    def stop_camera(self, data):
        self.camera_stop = True

    def callback(self, data):

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('wrong when converting message to cv2 image')
            print(e)

        print('frame shape: ' + str(frame.shape))
        print(frame[:,:,0])
        print('should be storingt img')
        #img_name = 'img/training_img' + str(self.counter) + '.png'
        #cv2.imshow(img_name, frame)
        #cv2.imwrite(os.path.join(os.getcwd(), img_name), frame)
        cv2.imwrite('img/image_' + str(self.counter) + '.jpeg', frame)
        #cv2.waitKey(1)

        #cv2.imwrite(img_name, frame)

        self.counter += 1
        '''
        print('frame shape: ' + str(frame.shape))

        if self.camera_rolling:
            self.counter += 1
            rospy.loginfo('Camera is rolling!!!')
            # catch the image frame

            # Initialize the writer which puts frames together in to a video file
            if self.writer is None:
                print('making new ghsdajhfdhfghd hgfha hdgf aygfsdka bg')
                # store the image dimensions, initialzie the video writer,
                # and construct the zeros array
                h, w = frame.shape[:2]
                # make sure arg three matches fps of stream input
                fps = 10  # make sure it is the same as the input stream otherwise it will have different speed
                self.writer = cv2.VideoWriter('Training_data', self.fourcc, fps,
                                              (w, h), True)

                # write the new file to the video file
            rospy.loginfo('I am writing to file')
            # Write the frame to file
            self.writer.write(frame)

        if self.counter > 50:
            0/0
        '''


if __name__ == '__main__':
    li = image_recorder()
    li.run()
