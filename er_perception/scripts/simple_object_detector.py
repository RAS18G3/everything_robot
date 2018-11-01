import cv2

##### Global variables ##################
SATURATION_THRESHOLD = 200 # decrease if red/orange/green/blue/yellow are not recognized
VIOLET_HUE_LOW = 120 # increase the violet range if violet is not recognized
VIOLET_HUE_HIGH = 140
VIOLET_SATURATION = 50 # decrease if violet is not recognized
# do the opposite if there are too many false positives

def find_bounding_boxes(img, debug = False):
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
    thresh5 = cv2.dilate(thresh5, cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10)), iterations=3)
    thresh5 = cv2.erode(thresh5, cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10)), iterations=2)

    if debug:
        # show the different masks, this can be helpful if the thresolds must be tuned
        cv2.imshow('mask 1', thresh1)
        cv2.imshow('mask 2', thresh2)
        cv2.imshow('mask 3', thresh3)
        cv2.imshow('mask 4', thresh4)
        cv2.imshow('mask 5', thresh5)

    # find contour of the final mask, as this can easily be used to find the bounding boxes
    _, contours, hierarchy = cv2.findContours(thresh5, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)


    # find the bounding boxes
    rectangles = []
    for contour in contours:
        rectangles.append(cv2.boundingRect(contour))


    if debug:
        # draw the bounding boxes and contours on the image
        cv2.drawContours(img, contours, -1, (255, 255, 0), 3)
        for rectangle in rectangles:
            cv2.rectangle(img, (rectangle[0], rectangle[1]), (rectangle[0] + rectangle[2], rectangle[1] + rectangle[3]),
            (0, 255, 0), 2)
        cv2.imshow('image', img)
        cv2.waitKey(1)

    return rectangles