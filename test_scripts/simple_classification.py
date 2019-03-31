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

def detectTrafficLight(sess,image_tensor,detect_boxes,detect_scores,detect_classes,num_detections,img_path,min_score_thresh):
    image = Image.open(img_path)
    (im_width, im_height) = image.size
    image_np = np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8) # load image into numpy array
    image_expanded = np.expand_dims(image_np, axis=0)

    (boxes, scores, classes, num) = sess.run(
        [detect_boxes, detect_scores, detect_classes, num_detections],
        feed_dict={image_tensor: image_expanded})
    
    # in my inference db, 2 refers to unknown
    classes[classes == 2.] = 4.

    # crop the traffic image
    for i in range(boxes.shape[1]):
        if scores[0,i] > min_score_thresh:
            ymin = int(boxes[0,i,0]*im_height)
            xmin = int(boxes[0,i,1]*im_width)
            ymax = int(boxes[0,i,2]*im_height)
            xmax = int(boxes[0,i,3]*im_width)
            traffic_img = image_np[ymin:ymax,xmin:xmax]
            cv2.imshow("ImageTraffic", cv2.cvtColor(traffic_img, cv2.COLOR_BGR2RGB))
            cv2.waitKey(0)
        else:
            break

    return (boxes, scores, classes, num, image_np)

detection_graph = load_graph(r'../models/frozen_inference_graph_traffic.pb')

label_map = label_map_util.load_labelmap(r'../data/merged_label_map.pbtxt')
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=13, use_display_name=True)
category_index = label_map_util.create_category_index(categories)
print(category_index)

test_images = glob(os.path.join(r'../data/test_sim', r'*.jpg'))
min_score_thresh=.5

with detection_graph.as_default():
    with tf.Session(graph=detection_graph) as sess:
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        detect_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        detect_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detect_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')
        
        for idx, img_path in enumerate(test_images):
            (boxes, scores, classes, num, image_np) = detectTrafficLight(sess,image_tensor,detect_boxes,detect_scores,detect_classes,num_detections,img_path,min_score_thresh)
            
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