from glob import glob
import os
import random
import tensorflow as tf
import numpy as np
from utils import label_map_util
from utils import visualization_utils as vis_util
from PIL import Image
import cv2

def load_graph(graph_file):
    """Loads a frozen inference graph"""
    graph = tf.Graph()
    with graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(graph_file, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    return graph

def create_feature(rgb_image):
    
    ## Convert image to HSV color space
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    
    ## Create and return a feature value and/or vector
    brightness_channel = hsv[:,:,2]
    rows = brightness_channel.shape[0]
    cols = brightness_channel.shape[1]
    mid = int(cols/2)
    
    red_region = brightness_channel[:int(rows/3),(mid-10):(mid+10)]
    yellow_region = brightness_channel[int(rows/3):int(2*rows/3),(mid-10):(mid+10)]
    green_region = brightness_channel[int(2*rows/3):,(mid-10):(mid+10)]
    
    feature = [0,0,0]
    feature[0] = np.mean(green_region)
    feature[1] = np.mean(red_region)
    feature[2] = np.mean(yellow_region)
    
    return feature

def predictTrafficClass(traffic_img):
    standard_im = cv2.resize(np.copy(traffic_img), (32, 32))
    rgb_image = cv2.cvtColor(standard_im, cv2.COLOR_BGR2RGB)
    max_index = np.argmax(create_feature(rgb_image))

    return max_index + 1.

def detectTrafficLight(sess,image_tensor,detect_boxes,detect_scores,detect_classes,num_detections,bgr_img,min_score_thresh):
    image = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
    im_height, im_width, channels = image.shape
    image_expanded = np.expand_dims(image, axis=0)

    (boxes, scores, classes, num) = sess.run(
        [detect_boxes, detect_scores, detect_classes, num_detections],
        feed_dict={image_tensor: image_expanded})
    
    # in my inference db, 2 refers to unknown
    classes[classes == 2.] = 4.

    print(classes.shape)
    # crop the traffic image
    for i in range(boxes.shape[1]):
        if scores[0,i] > min_score_thresh:
            ymin = int(boxes[0,i,0]*im_height)
            xmin = int(boxes[0,i,1]*im_width)
            ymax = int(boxes[0,i,2]*im_height)
            xmax = int(boxes[0,i,3]*im_width)

            # for efficiency, I only process images which have high confidence score
            if ((ymax - ymin) * (xmax - xmin)) >= 1024:
                traffic_img = image[ymin:ymax,xmin:xmax]
                classes[0,i] = predictTrafficClass(traffic_img)
        else:
            break

    return (boxes, scores, classes, num, image)

detection_graph = load_graph(r'../models/frozen_inference_graph_traffic_130.pb')

label_map = label_map_util.load_labelmap(r'../data/merged_label_map.pbtxt')
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=13, use_display_name=True)
category_index = label_map_util.create_category_index(categories)
print(category_index)

test_images = glob(os.path.join(r'../data/test_simcapstone', r'*.jpg'))
min_score_thresh=.5

with detection_graph.as_default():
    with tf.Session(graph=detection_graph) as sess:
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        detect_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        detect_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detect_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')
        
        for idx, img_path in enumerate(test_images):
            bgr_img = cv2.imread(img_path) # bgr
            (boxes, scores, classes, num, image_np) = detectTrafficLight(sess,image_tensor,detect_boxes,detect_scores,detect_classes,num_detections,bgr_img,min_score_thresh)
            
            print('Probabilities')
            print(scores[0])
            print('Classes: 1 = Green, 2 = Red, 3 = Yellow, 4 = Unknown')
            print(classes[0])

            vis_util.visualize_boxes_and_labels_on_image_array(
                image_np, 
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                category_index,
                use_normalized_coordinates=True,
                max_boxes_to_draw=5,
                min_score_thresh=min_score_thresh,
                line_thickness=8)
            cv2.imshow("Image", cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB))
            cv2.waitKey(0)