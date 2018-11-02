import numpy as np
from simple_object_detector import find_bounding_boxes
import os
import cv2
from os import listdir
from os.path import isfile, isdir, join


# dataset should be in the following format
# dataset/0_class_name_0/raw/1.jpg
# dataset/0_class_name_0/raw/2.jpg
# dataset/0_class_name_0/raw/3.jpg
# dataset/1_class_name_1/raw/1.jpg
# dataset/1_class_name_1/raw/2.jpg
# ...

# running this script in SIMPLE_OBJECT_DETECTOR mode will run the simple object detector to find ROI in the image will create the follow folders/files
# mxn is the inputsize
# dataset/0_class_name_0/mxn/1.jpg
# dataset/0_class_name_0/mxn/2.jpg
# dataset/0_class_name_0/mxn/3.jpg
# dataset/1_class_name_1/mxn/1.jpg
# dataset/1_class_name_1/mxn/2.jpg
# ...

# all ROIs will be resized to this size
inputsize = (32, 32)

# how much percent of the images will be used for validation
validation_percentage = 0.2

mode = "SIMPLE_OBJECT_DETECTOR"

# create test / training data by running the whole computer vision pipeline
if mode == "SIMPLE_OBJECT_DETECTOR":
    # dataset folder assumed to be in same folder as script
    dirname = os.path.dirname(__file__)

    # onlyfiles = [f for f in listdir(mypath) if isfile(join(dirname, f))]
    dataset_path = os.path.join(dirname, 'dataset')

    print(listdir(dataset_path))

    class_folder_names = [dir for dir in listdir(dataset_path) if isdir(os.path.join(dataset_path, dir))]

    class_names = []
    class_paths = []

    class_idx = 0
    current_idx = 0
    while True:
        if current_idx >= len(class_folder_names):
            break
        if class_folder_names[current_idx].startswith(str(class_idx)+'_'):
            class_paths.append(os.path.join(dataset_path, class_folder_names[current_idx]))
            class_names.append(class_folder_names[current_idx][len(str(class_idx)+'_'):])
            current_idx = 0
            class_idx +=1
        else:
            current_idx += 1

    class_idx = 0
    count = 0
    for class_path in class_paths:
        out_dir = os.path.join(class_path, '{0}x{1}'.format(inputsize[0], inputsize[1]))
        try:
            os.mkdir(out_dir)
        except:
            pass
        raw_image_files = [file for file in listdir(os.path.join(class_path, "raw")) if isfile(os.path.join(class_path, "raw", file))]

        for raw_image_file in raw_image_files:
            raw_image_path = os.path.join(class_path, 'raw', raw_image_file)
            img = cv2.imread(raw_image_path)

            rectangles = find_bounding_boxes(img)
            for rectangle in rectangles:
                (x,y,w,h) = rectangle
                roi = img[y:y+h, x:x+w]
                roi_resized = cv2.resize(roi, inputsize)

                cv2.imwrite(os.path.join(out_dir, str(count)+'.jpeg'), roi_resized)

                cv2.imshow('image', roi_resized)
                cv2.waitKey(1)
                count += 1

        class_idx += 1


if mode == "TRAIN":
    pass
