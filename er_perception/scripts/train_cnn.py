import numpy as np
from simple_object_detector import find_bounding_boxes
import os
from os import listdir
from os.path import isfile, isdir, join


# dataset should be in the following format
# dataset/1_class_name_1/raw/1.jpg
# dataset/1_class_name_1/raw/2.jpg
# dataset/1_class_name_1/raw/3.jpg
# dataset/2_class_name_2/raw/1.jpg
# dataset/2_class_name_2/raw/2.jpg
# ...

# running this script in SIMPLE_OBJECT_DETECTOR mode will run the simple object detector to find ROI in the image will create the follow folders/files
# mxn is the inputsize
# dataset/1_class_name_1/mxn/1.jpg
# dataset/1_class_name_1/mxn/2.jpg
# dataset/1_class_name_1/mxn/3.jpg
# dataset/2_class_name_2/mxn/1.jpg
# dataset/2_class_name_2/mxn/2.jpg
# ...

# all ROIs will be resized to this size
inputsize = (32, 32)

mode = "SIMPLE_OBJECT_DETECTOR"

# create test / training data by running the whole computer vision pipeline
if mode == "SIMPLE_OBJECT_DETECTOR":
    # dataset folder assumed to be in same folder as script
    dirname = os.path.dirname(__file__)

    onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
    dataset_path = os.path.join(dirname, 'dataset')
    print(listdir(dataset_path))

if mode == "TRAIN":
    pass
