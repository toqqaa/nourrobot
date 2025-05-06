#!/usr/bin/env python3

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String

class HumanDetectionPublisher:
    def __init__(self):
        rospy.init_node('human_detection_publisher')
        
        # Publisher for robot control messages
        self.control_pub = rospy.Publisher('/robot_control', String, queue_size=1)
        
        # Subscribe to darknet_ros bounding boxes
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detection_callback)
        
        rospy.loginfo("Human Detection Publisher initialized")

    def detection_callback(self, data):
        # Check if any detected object is a person
        for box in data.bounding_boxes:
            if box.Class == "person":  # Assuming darknet_ros is configured to detect "person" class
                # Publish stop message when human detected
                control_msg = String()
                control_msg.data = "stop"
                self.control_pub.publish(control_msg)
                rospy.loginfo("Human detected -  stop robot")
                break

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = HumanDetectionPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass