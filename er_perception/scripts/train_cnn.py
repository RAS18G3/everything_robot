import numpy as np
from simple_object_detector import find_bounding_boxes
import os
import cv2
from os import listdir
from os.path import isfile, isdir, join
from keras import layers
from keras import models
from keras.utils import to_categorical
import numpy as np
import matplotlib.pyplot as plt


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

mode = "TRAIN"

# create test / training data by running the whole computer vision pipeline
if mode == "SIMPLE_OBJECT_DETECTOR":
    # dataset folder assumed to be in same folder as script
    dirname = os.path.dirname(__file__)

    dataset_path = os.path.join(dirname, 'dataset')

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
    def build_model():
        model = models.Sequential()
        model.add(layers.Conv2D(32, (3, 3), activation='relu', input_shape=(inputsize[0], inputsize[1], 3)))
        model.add(layers.MaxPooling2D((2,2)))
        model.add(layers.Conv2D(256, (3, 3), activation='relu'))
        model.add(layers.MaxPooling2D((2,2)))
        model.add(layers.Conv2D(256, (3,3), activation='relu'))
        model.add(layers.Flatten())
        model.add(layers.Dense(64, activation='relu'))
        model.add(layers.Dense(15, activation='softmax'))
        model.summary()
        return model

    def load_data():
        dirname = os.path.dirname(__file__)
        dataset_path = os.path.join(dirname, 'dataset')
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
        training_data = []
        training_labels = []
        for class_path in class_paths:
            image_dir = os.path.join(class_path, '{0}x{1}'.format(inputsize[0], inputsize[1]))
            image_files = [file for file in listdir(image_dir) if isfile(os.path.join(image_dir, file))]
            for image_file in image_files:
                img = cv2.imread(os.path.join(image_dir, image_file))
                # img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                # print(os.path.join(image_dir, image_file))
                # print(img.shape)
                training_data.append(np.asarray(img))
                training_labels.append(to_categorical(class_idx, num_classes=15))
            class_idx += 1

        # print(training_labels)
        training_data = np.asarray(training_data, dtype=np.float32) / 255
        training_labels = np.asarray(training_labels, dtype=np.float32)
        return (training_data, training_labels)

    model = build_model()
    (data, labels) = load_data()

    shuffled_indices = np.arange(data.shape[0])
    np.random.shuffle(shuffled_indices)
    data = data[shuffled_indices]
    labels = labels[shuffled_indices]
    validation_percentage = 0.9
    split = int(validation_percentage * data.shape[0])
    training_data = data[:split]
    val_data = data[split:]
    training_labels = labels[:split]
    val_labels = labels[split:]
    print(data.shape)
    print(labels.shape)

    model.compile(optimizer='rmsprop',
                  loss='categorical_crossentropy',
                  metrics=['accuracy'])
    history = model.fit(training_data, training_labels, epochs=200, batch_size=64, validation_data=(val_data, val_labels))

    # test_loss, test_acc = model.evaluate(test_data, test_labels)
    # print(test_loss, test_acc)

    acc_values = history.history['acc']
    val_acc_values = history.history['val_acc']
    epochs = range(1, len(acc_values) + 1)
    plt.plot(epochs, acc_values, 'bo', label='Training acc')
    plt.plot(epochs, val_acc_values, 'b', label='Validation acc')
    plt.title('Training and validation accuracy')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.legend()
    plt.show()

    model.save('cnn.h5')
