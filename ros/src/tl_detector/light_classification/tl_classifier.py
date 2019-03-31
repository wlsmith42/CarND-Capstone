from styx_msgs.msg import TrafficLight
from glob import glob
from PIL import Image
import label_map_util
import cv2
import os
import random
import tensorflow as tf
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #load classifier
	graph_file = '../../../models/frozen_inference_graph.pb'
        self.graph = tf.Graph()

	with self.graph.as_default():
	    od_graph_def = tf.GraphDef()

	    with tf.gfile.GFile(graph_file, 'rb') as fid:
		serialized_graph = fid.read()
		od_graph_def.ParseFromString(serialized_graph)
		tf.import_graph_def(od_graph_def, name='')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #light color prediction
	with self.graph.as_default():

	    with tf.Session(graph=self.graph) as sess:
		image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
		detect_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
		detect_scores = self.graph.get_tensor_by_name('detection_scores:0')
		detect_classes = self.graph.get_tensor_by_name('detection_classes:0')
		num_detections = self.graph.get_tensor_by_name('num_detections:0')

		image_expanded = np.expand_dims(image, axis=0)

		(boxes, scores, classes, num) = sess.run(
		    [detect_boxes, detect_scores, detect_classes, num_detections],
		    feed_dict={image_tensor: image_expanded})

		classification = classes[0][0]
		probability = scores[0][0]

		if(classification == 1.0):
		    print('GREEN - ', probability)
		    return TrafficLight.GREEN

		elif(classification == 2.0):
		    print('RED - ', probability)
		    return TrafficLight.RED

		elif(classification == 3.0):
		    print('YELLOW - ', probability)
		    return TrafficLight.YELLOW


		
        return TrafficLight.UNKNOWN
