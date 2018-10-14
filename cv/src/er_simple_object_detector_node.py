#!/usr/bin/env python
import roslib
roslib.load_manifest('cv')
#import sys
import rospy
import cv2
#from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

'''
Do not forget to source worskpace every time opening a new terminal
source ./delev/setup.bash
Do not forget to make python file executable everytime creating a new pyhon node
chmod +x file_name.py
'''

##### Global variables ##################

# Yellow
YELLOW_MIN = np.array([20, 180, 180],np.uint8)
YELLOW_MAX = np.array([30, 255, 255],np.uint8)

# RED AND ORANGE
RED_ORANGE_MIN_LOW = np.array([0, 180, 180],np.uint8)
RED_ORANGE_MAX_LOW = np.array([15, 255, 255],np.uint8)

RED_ORANGE_MIN_HIGH = np.array([159, 180, 180],np.uint8)
RED_ORANGE_MAX_HIGH = np.array([179, 255, 255],np.uint8)

# RED AND ORANGE
RED_MIN = np.array([0, 150, 180],np.uint8)
RED_MAX = np.array([5, 255, 255],np.uint8)

# RED AND ORANGE
ORANGE_MIN = np.array([7, 150, 180],np.uint8)
ORANGE_MAX = np.array([15, 255, 255],np.uint8)

# Blue/ cyan
BLUE_MIN = np.array([85, 180, 120],np.uint8)
BLUE_MAX = np.array([105, 255, 255],np.uint8)

# Green
GREEN_MIN = np.array([40, 180, 100],np.uint8)
GREEN_MAX = np.array([60, 255, 255],np.uint8)


####### Helper functions ################

def resize(image, width = None, height = None, inter = cv2.INTER_AREA):
    dim = None
    h, w = image.shape[:2]

    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    resized = cv2.resize(image, dim, interpolation=inter)
    return resized

def mask_image_for_colors(hsv_img):
    img_mask_yellow = cv2.inRange(hsv_img, YELLOW_MIN, YELLOW_MAX)


    # Mask on both the high and low end of the hue vlue span since red - orange goes from h app 159 -> h app 15
    img_thresh_ored_low = cv2.inRange(hsv_img, RED_ORANGE_MIN_LOW, RED_ORANGE_MAX_LOW)
    img_thresh_ored_high = cv2.inRange(hsv_img, RED_ORANGE_MIN_HIGH, RED_ORANGE_MAX_HIGH)
    img_mask_ored = cv2.bitwise_or(img_thresh_ored_low, img_thresh_ored_high)

    img_mask_blue = cv2.inRange(hsv_img, BLUE_MIN, BLUE_MAX)

    img_mask_green = cv2.inRange(hsv_img, GREEN_MIN, GREEN_MAX)

    img_mask_red = cv2.inRange(hsv_img, RED_MIN, RED_MAX)

    img_mask_orange = cv2.inRange(hsv_img, ORANGE_MIN, ORANGE_MAX)

    # For masking orange and red separately not needed?
    #masked_images = [img_mask_yellow, img_mask_blue, img_mask_green, img_mask_red, img_mask_orange]
    # Red and orange are masked as one color
    masked_images = [img_mask_yellow, img_mask_ored, img_mask_blue, img_mask_green]

    #masked_images = [img_mask_ored]

    return masked_images

def detect_objects(masked_imgs):

    objects = []
    for masked_img in masked_imgs:

        edges = cv2.Canny(masked_img, 100, 200)
        contours = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

        # Check the size of contour, so it is not just a fex single points
        for contour in contours:
            # possible to set a threshold value for this to remove very small contours objects
            peri = cv2.arcLength(contour, True)

            # Find good values for len of contours and peri
            if len(contour) > 6 and peri > 70:
                objects.append(contour)

    return objects

class simple_object_detector_node:

    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('er_simple_object_detector_node', anonymous=True)

        # Change to publish bounding boxes
        self.image_pub = rospy.Publisher("object_location", Image)
        # Needed to convert Image message to cv2 image type
        self.bridge = CvBridge()

        # Subscriber listening for the images sent by the camera, and set the callback function
        #self.image_sub = rospy.Subscriber('camera/rgb/image_rect_color', Image, self.detect_object)

        self.image_sub = rospy.Subscriber('camera/rgb/image_rect_color', Image, self.detect_object)

    def run(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo('I have started!!!')
        rospy.spin()

    def callback(self, data):
        print('I am receiving data!')

    def detect_object(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('wrong when converting message to cv2 image')
            print(e)

        # Apply smoothing to the image
        img = cv2.medianBlur(cv_image, 9)

        # Convert to hsv color space for better illumination robustness
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Get a list of all the masked images
        masked_images = mask_image_for_colors(hsv_img)

        # Try and detect the objects from the masked images
        objects = detect_objects(masked_images)

        boxes = []
        for object in objects:
            box = cv2.boundingRect(object)
            boxes.append(box)

        for box in boxes:
            (x,y,w,h) = box
            cv2.rectangle(cv_image, (x,y), (x+w, y+h), (0,255,0), 1)
        cv2.imshow('Object_detector', cv_image)
        cv2.waitKey(1)

        l = len(boxes)


        print('Found {} object/s in the image'.format(l))
        # Should publish the boxes

'''
class test:

    def __init__(self):
        rospy.init_node('er_simple_object_detector_node', anonymous=True)

        self.image_sub = rospy.Subscriber('camera/rgb/image_rect_color', Image, self.callback)

    def run(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo('I have started!!!')
        rospy.spin()

    def callback(self, data):
        rospy.loginfo('I have started!!!')
'''

if __name__ == '__main__':
    li = simple_object_detector_node()
    #li = test()
    li.run()
