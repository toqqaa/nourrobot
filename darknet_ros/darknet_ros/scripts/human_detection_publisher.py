#!/usr/bin/env python3
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os


class HumanDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.detection_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detection_callback)
        self.control_pub = rospy.Publisher('/ai/robot_control', String, queue_size=1)
        self.depth_image = None
        self.current_state = "continue"  # Track current state
        self.safety_distance = 1.0  # Default safety distance (meters)
        self.last_person_detected_time = rospy.Time.now()
        self.person_timeout = rospy.Duration(4.0)  # 4 second timeout, adjustable based on FPS
        
        # Timer to check for timeout, running at 1Hz
        rospy.Timer(rospy.Duration(1.0), self.check_person_timeout)
        
        # Load configuration
        self.load_config()

    def load_config(self):
        config_path = os.getenv('DISTANCE_CONFIG_PATH', '/home/jetson/noor_ws/darknet_ros/darknet_ros/config/distance_config.json')
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
                self.safety_distance = config.get('safety_distance', 1.0)
                rospy.loginfo(f"Loaded safety distance: {self.safety_distance}m")
        except Exception as e:
            rospy.logwarn(f"Failed to load config file: {e}")
            rospy.logwarn("Using default safety distance: 1.0m")
            self.safety_distance = 1.0

    def depth_callback(self, depth_msg):
        """
        Callback function for the depth image topic.
        """
        try:
            depth_array = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            # Convert depth to meters
            depth_array = depth_array.astype(np.float32) / 1000.0
            self.depth_image = depth_array
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")

    def get_depth_value(self, x, y, width, height):
        """
        Get the median depth value in the region of interest.
        """
        if self.depth_image is None:
            return None
            
        # Define the region of interest (ROI)
        roi_x_start = max(0, x - width // 4)
        roi_x_end = min(self.depth_image.shape[1], x + width // 4)
        roi_y_start = max(0, y - height // 4)
        roi_y_end = min(self.depth_image.shape[0], y + height // 4)
        roi = self.depth_image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

        # Extract valid depth values
        valid_depths = roi[np.logical_and(roi > 0.1, roi < 5.0)]
        if len(valid_depths) > 0:
            return np.median(valid_depths)
        return None

    def check_person_timeout(self, event):
        """
        Check if we haven't detected a person within safety distance for a while
        """
        if self.current_state == "stop" and (rospy.Time.now() - self.last_person_detected_time) > self.person_timeout:
            rospy.loginfo("No person detected within safety distance for timeout period - CONTINUE")
            self.control_pub.publish("continue")
            self.current_state = "continue"

    def detection_callback(self, detection_msg):
        """
        Callback function for the bounding box detections.
        """
        if self.depth_image is None:
            return

        person_in_range = False

        # Process each bounding box
        for box in detection_msg.bounding_boxes:
            if box.Class == "person":
                width = box.xmax - box.xmin
                height = box.ymax - box.ymin
                center_x = (box.xmin + box.xmax) // 2
                center_y = (box.ymin + box.ymax) // 2

                depth = self.get_depth_value(center_x, center_y, width, height)

                if depth is not None and depth <= self.safety_distance:
                    person_in_range = True
                    self.last_person_detected_time = rospy.Time.now()
                    if self.current_state != "stop":
                        rospy.loginfo(f"Person detected within safety distance: {depth:.2f}m - STOP")
                        self.control_pub.publish("stop")
                        self.current_state = "stop"
                    break  # Stop checking once a person in range is found

        # If no person is in range and we're in stop state, let the timeout handle the transition
        # If no person is in range and we're already in continue state, do nothing
        if not person_in_range and self.current_state == "continue":
            # This is just to maintain the existing state
            pass


if __name__ == '__main__':
    rospy.init_node('human_distance_detector')
    detector = HumanDetector()
    rospy.spin()
