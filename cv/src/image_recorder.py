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
        rospy.Subscriber("camera/rgb/image_rect_color", Bool, self.start_camera)

        rospy.Subscriber("camera_start", Bool, self.start_camera)
        rospy.Subscriber("stop_camera", Bool, self.stop_camera)

        self.camera_rolling = False
        self.camera_stop = False
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.writer = None

    def run(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo('I have started!!!')
        rospy.spin()

    def start_camera(self, data):
        self.camera_rolling = data.data

    def stop_camera(self, data):
        self.camera_stop = True

    def callback(self, data):
        if self.camera_rolling:
            rospy.loginfo('Camera is rolling!!!')
            # catch the image frame
            frame = np.array(data.data)

            # Initialize the writer which puts frames together in to a video file
            if self.writer is None:
                # store the image dimensions, initialzie the video writer,
                # and construct the zeros array
                (h, w) = frame.shape[:2]
                # make sure arg three matches fps of stream input
                fps = 10  # make sure it is the same as the input stream otherwise it will have different speed
                self.writer = cv2.VideoWriter('Training_data', self.fourcc, fps,
                                              (w, h), True)

                # write the new file to the video file
            rospy.loginfo('I am writing to file')
            # Write the frame to file
            self.writer.write(frame)


if __name__ == '__main__':
    li = image_recorder()
    li.run()