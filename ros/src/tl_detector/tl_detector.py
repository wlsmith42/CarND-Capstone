#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import tensorflow
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        self.frame_drop = -1
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.waypoints_2d = None
        self.waypoint_tree = None

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub4 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=2*52428800)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        #ROS Topic that redlight data will be published to
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        
        #Variable set through ROS launch config file to determine whether to load the simulator or real world classifier
        is_site = self.config['is_site']
        if is_site:
            #Load the real world classifier trained in TensorFlow 1.3.0
            graph_file = '../../../models/frozen_inference_graph_real_merged_130.pb'
        else:
            #Load the simulator clasifier trained in TensorFlow 1.3.0
            graph_file = '../../../models/frozen_inference_graph_traffic_130.pb'
            
        #Pre-load tensorflow graph to speed up processing
        self.graph = tensorflow.Graph()

        self.bridge = CvBridge()
        #Create classifier object
        self.light_classifier = TLClassifier(self.graph, graph_file)
        self.listener = tf.TransformListener()
        #Pre-load tensorflow session to further speed up processing
        self.sess = tensorflow.Session(graph=self.graph)

        #Traffic light state information
        self.state = None
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        #Position callback
        #Gets updated location of the vehicle from '/current_pose' ROS Topic 
        self.pose = msg

    def waypoints_cb(self, waypoints):
        #Waypoints callback
        #Gets updated list of waypoints from '/base_waypoints' ROS Topic
        self.waypoints = waypoints

        #Load waypoints into a KDTree to quickly find the closest waypoint later on
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        #Traffic light callback
        #Gets information about trafflic lights from the simulator that are published to the '/vehicle/traffic_lights' ROS Topic
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        #if the process_traffic_light function rejects the image, drop everything and don't publish an update
        if light_wp == -1 and state == TrafficLight.UNKNOWN:
            return
        
        #The following code block ensures that false-positives do not get published to the ROS Topic
        try:
            #If there is no new state, increment the counter
            if self.state != state:
                self.state_count = 0
                self.state = state
                
            #If the state has occurred the set amount of times, it can be trusted so publish the new data
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
                
            #Otherwise, continue to publish the previously confirmed state
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1

        except AttributeError:
            #If the state information for the tl_detector has not fully loaded this will cause an error
            #This except catches any runtime error that occurs due to a not fully loaded component of this file
            print "tl_detector still loading..."
            return


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        min_loc = -1

        #Make sure the KDTree is loaded and initialized
        if pose is not None and self.waypoint_tree is not None:
            #Find the nearest waypoint to the position using a KDTree query
            min_loc = self.waypoint_tree.query([pose.position.x,pose.position.y], 1)[1]

        return min_loc

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #Check if there is an image before trying to classify it
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        #Convert the image to cv
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        try:
            #Run the image through the Traffic Light Classifier using the current TensorFlow session
            classification = self.light_classifier.get_classification(self.sess, cv_image)

        except AttributeError:
            #If the Tensorflow session has not finished loading, this except catches the error and will allow the tl_detector to wait for Tensorflow to load
            print "tl_classifier still loading..."
            return False

        return classification

    
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #Only run the tl_detector on every 5th image to reduce simulator latency
        self.frame_drop += 1
        if self.frame_drop > 0:
            if self.frame_drop == 4:
                self.frame_drop = -1

            return -1, TrafficLight.UNKNOWN


        light = None
        closest_ls_wp = None
        dist_to_light = 9999

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
        else:
            return -1, TrafficLight.UNKOWN


        #TODO find the closest visible traffic light (if one exists)
        for stop_line_position in stop_line_positions:
            sl_pose = Pose()
            sl_pose.position.x = stop_line_position[0]
            sl_pose.position.y = stop_line_position[1]

            ls_wp = self.get_closest_waypoint(sl_pose)

            #Check if the closest waypoint is in front of the car
            if ls_wp >= car_position:

                #if there is no closest waypoint, or it is larger than the current waypoint, use the current waypoint
                if closest_ls_wp is None or ls_wp < closest_ls_wp:
                    closest_ls_wp = ls_wp
                    light = sl_pose

                #Calculate the distance from the car to the closest waypoint
                if car_position is not None and closest_ls_wp is not None:
                    dist_to_light = abs(car_position - closest_ls_wp)

        #Only get the light state if the car is close to a light to reduce processing
        if light and dist_to_light < 80:
            state = self.get_light_state(light)
            return closest_ls_wp, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
