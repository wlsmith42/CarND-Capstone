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
    def __init__(self, graph, graph_file):
        #load classifier
        self.graph = graph

        with self.graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
    
    def create_feature(self,rgb_image):
        
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
    
    def predictTrafficClass(self,traffic_img):
        standard_im = cv2.resize(np.copy(traffic_img), (32, 32))
        rgb_image = cv2.cvtColor(standard_im, cv2.COLOR_BGR2RGB)
        max_index = np.argmax(self.create_feature(rgb_image))

        return max_index + 1.
    
    def detectTrafficLight(self,sess,image_tensor,detect_boxes,detect_scores,detect_classes,num_detections,bgr_img,min_score_thresh):
        image = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        im_height, im_width, channels = image.shape
        image_expanded = np.expand_dims(image, axis=0)

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

                # for efficiency, I only process images which have high confidence score
                if ((ymax - ymin) * (xmax - xmin)) >= 1024:
                    traffic_img = image[ymin:ymax,xmin:xmax]
                    classes[0,i] = self.predictTrafficClass(traffic_img)
            else:
                break

        return (boxes, scores, classes, num, image)

    def get_classification(self, sess, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #light color prediction
        with self.graph.as_default():

            image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            detect_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            detect_scores = self.graph.get_tensor_by_name('detection_scores:0')
            detect_classes = self.graph.get_tensor_by_name('detection_classes:0')
            num_detections = self.graph.get_tensor_by_name('num_detections:0')

            min_score_thresh=.5
            (boxes, scores, classes, num, image_np) = self.detectTrafficLight(sess,image_tensor,detect_boxes,detect_scores,detect_classes,num_detections,image,min_score_thresh)

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
