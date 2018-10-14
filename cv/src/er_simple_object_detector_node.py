#!/usr/bin/env python
import roslib
roslib.load_manifest('cv')
#import sys
import rospy
import cv2
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

'''
Do not forget to source worskpace e
very time opening a new terminal
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

def overlapping_boxes(box1, box2):
    # delta overlap is for detection boxes very close to each other so that they can be merged in to one
    delta_overlap = 5

    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2

    overlap_x = False
    overlap_y = False

    # Check if the x values of the boxes overlap
    if x1 - delta_overlap < x2 < (x1 + w1 + delta_overlap):
        overlap_x = True
    elif x1 - delta_overlap < (x2 + w2) < (x1 + w1 + delta_overlap):
        overlap_x = True
    elif x2 - delta_overlap < x1 < (x2 + w2 + delta_overlap):
        overlap_x = True

    # Check if the y values of the boxes overlap
    if y1 - delta_overlap < y2 < (y1 + h1 + delta_overlap):
        overlap_y = True
    elif y1 - delta_overlap < (y2 + h2) < (y1 + h1 + delta_overlap):
        overlap_y = True
    elif y2 - delta_overlap < y1 < (y2 + h2 + delta_overlap):
        overlap_y = True

    # if both the x and y values overlap the boxes are overlapping
    if overlap_x and overlap_y:
        return True
    else:
        return False


def box_merge(box1, box2):
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2

    x = min(x1, x2)
    y = min(y1, y2)

    x_max = max(x1 + w1, x2 + w2)
    y_max = max(y1 + h1, y2 + h2)

    w = x_max - x
    h = y_max - y

    return (x, y, w, h)


def box_merge(boxes):
    min_x, min_y, w, h = boxes[0]
    max_x = min_x + w
    max_y = min_y + h

    for box in boxes:
        x, y, w, h = box
        x2 = x + w
        y2 = y + h

        if min_x > x:
            min_x = x
        if min_y > y:
            min_y = y
        if max_x < x2:
            max_x = x2
        if max_y < y2:
            max_y = y2

    w = max_x - min_x
    h = max_y - min_y

    return (min_x, min_y, w, h)


def merge_boxes(boxes):
    i = 0

    while (i < len(boxes)):
        box = boxes[i]
        over_boxes = []
        l = len(boxes)
        index_overlapping_boxes = []
        for j in range(i + 1, l):
            if overlapping_boxes(box, boxes[j]):
                over_boxes.append(boxes[j])
                index_overlapping_boxes.append(j)

        # If the box overlapped with other boxes they should all be merged
        if len(over_boxes) > 0:
            over_boxes.append(box)
            index_overlapping_boxes.append(i)

            new_boxes = boxes[:i]
            merged_box = box_merge(over_boxes)
            new_boxes.append(merged_box)

            for k in range(i + 1, len(boxes)):
                if k not in index_overlapping_boxes:
                    new_boxes.append(boxes[k])

            boxes = new_boxes
        # If the first box did not overlap with anny boxes move on anf check if the next box overaps with any of the other boxes
        else:
            i += 1

    return boxes

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

    for masked_img in masked_imgs:

        edges = cv2.Canny(masked_img, 100, 200)
        contours = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

        boxes = [cv2.boundingRect(object) for object in contours]
        boxes = merge_boxes(boxes)

        # Find good values for volume and w h ratio
        prediction_boxes = []
        for box in boxes:
            x,y,w,h = box
            if not (w/h > 3 or h/w > 3) and (w*h > 1000):
                prediction_boxes.append(box)


    return prediction_boxes

class simple_object_detector_node:

    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('er_simple_object_detector_node', anonymous=True)

        # Change to publish bounding boxes
        self.box_publisher = rospy.Publisher("object_bounding_boxes", UInt16MultiArray)
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
        boxes = detect_objects(masked_images)

        #boxes = []
        #for object in objects:
        #    box = cv2.boundingRect(object)
        #    boxes.append(box)

        # The message which will be sent
        bounding_box_msg = UInt16MultiArray()
        for box in boxes:
            (x,y,w,h) = box
            bounding_box_msg.data.append(x)
            bounding_box_msg.data.append(y)
            bounding_box_msg.data.append(w)
            bounding_box_msg.data.append(h)

            cv2.rectangle(cv_image, (x,y), (x+w, y+h), (0,255,0), 1)
        cv2.imshow('Object_detector', cv_image)
        cv2.waitKey(1)

        l = len(boxes)

        self.box_publisher.publish(bounding_box_msg)
        print('published the following bounding boxes: ' + str(bounding_box_msg))
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
