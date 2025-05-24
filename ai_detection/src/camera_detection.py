#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from deepface import DeepFace
import os

class DeepFaceAnalyzer:
    def __init__(self):
        # Initialize ROS components
        self.bridge = CvBridge()
        self.frame_count = 0
        self.analysis_interval = 3  # Analyze every 3 frames
        
        # Configure DeepFace parameters
        self.analysis_params = {
            'actions': ['gender', 'emotion', 'age'],
            'enforce_detection': False,
            'detector_backend': 'opencv',
            'align': True
        }
        
        # Initialize face cascade
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        
        # Start subscriber
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.loginfo("DeepFace Analyzer initialized - Ready for processing")

    def analyze_face(self, frame, face_roi):
        """Perform DeepFace analysis on face region"""
        try:
            # Convert ROI to RGB (DeepFace expects RGB)
            face_rgb = cv2.cvtColor(face_roi, cv2.COLOR_BGR2RGB)
            
            # Perform analysis (only specified actions)
            results = DeepFace.analyze(
                img_path=face_rgb,
                **self.analysis_params
            )
            
            # Extract most confident results
            if isinstance(results, list):
                results = results[0]
            
            gender = results['gender']
            emotion = results['dominant_emotion']
            age = int(results['age'])
            
            # Post-process gender for better display
            gender = self._format_gender(gender)
            
            return gender, emotion, age
            
        except Exception as e:
            rospy.logwarn(f"DeepFace analysis warning: {str(e)}")
            return "Unknown", "Unknown", 0

    def _format_gender(self, gender_result):
        """Format gender output for better readability"""
        if isinstance(gender_result, dict):
            # Return the dominant gender if in dict format
            return max(gender_result.items(), key=lambda x: x[1])[0]
        return str(gender_result)

    def image_callback(self, msg):
        """Process each camera frame"""
        self.frame_count += 1
        
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            display_frame = frame.copy()
            
            # Face detection (simpler than DeepFace's for performance)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(
                gray, 
                scaleFactor=1.1, 
                minNeighbors=5, 
                minSize=(100, 100)
            )
            
            for (x, y, w, h) in faces:
                # Get face ROI with padding
                padding = 20
                x1, y1 = max(0, x-padding), max(0, y-padding)
                x2, y2 = min(frame.shape[1]-1, x+w+padding), min(frame.shape[0]-1, y+h+padding)
                face_roi = frame[y1:y2, x1:x2]
                
                # Only analyze periodically for performance
                if self.frame_count % self.analysis_interval == 0:
                    gender, emotion, age = self.analyze_face(frame, face_roi)
                else:
                    # Reuse previous results between analyses
                    if not hasattr(self, 'last_results'):
                        continue
                    gender, emotion, age = self.last_results
                
                # Store results for reuse
                self.last_results = (gender, emotion, age)
                
                # Draw results
                label = f"{gender}, {age}, {emotion}"
                cv2.rectangle(display_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(display_frame, label, (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display output
            cv2.imshow("DeepFace Analysis", display_frame)
            key = cv2.waitKey(1)
            if key == ord('q'):
                rospy.signal_shutdown("User requested shutdown")
                
        except Exception as e:
            rospy.logerr(f"Frame processing error: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('deepface_analyzer')
    
    # Configure to use less verbose DeepFace logging
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
    
    try:
        analyzer = DeepFaceAnalyzer()
        rospy.loginfo("Starting DeepFace analysis node...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
        rospy.loginfo("Node shutdown complete")