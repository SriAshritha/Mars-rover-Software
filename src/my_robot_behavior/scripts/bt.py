#!/usr/bin/env python3
import rospy
import rospkg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import time
import os

class NodeStatus:
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"

class Node:
    def run(self):
        raise NotImplementedError("run() must be implemented in subclass")

class SequenceNode(Node):
    def __init__(self, children):
        self.children = children

    def run(self):
        for child in self.children:
            status = child.run()
            if status != NodeStatus.SUCCESS:
                return status
        return NodeStatus.SUCCESS

class ArrowDetection:
    def __init__(self):
        self.detected = False
        self.direction = None
        self.right_arrow = None
        self.left_arrow = None
        self.cap = None

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('arrow_detection')   
        right_arrow_path = os.path.join(package_path, "src/right_arrow.png")
        left_arrow_path = os.path.join(package_path, "src/left_arrow.png")
        self.right_arrow = cv2.imread(right_arrow_path, cv2.IMREAD_GRAYSCALE)
        self.left_arrow = cv2.imread(left_arrow_path, cv2.IMREAD_GRAYSCALE)

        if self.right_arrow is None or self.left_arrow is None:
            rospy.logerr("Failed to load arrow templates.")

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Cannot open webcam.")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            rospy.logerr("Error capturing frame.")
            return False, None

        # Simplified detection logic
        contours_detected = self.detect_contours(frame)
        if contours_detected:
            return True, "Detected"
        if self.match_and_annotate(frame, self.right_arrow, (0, 255, 0), "Right"):
            return True, "Right"
        if self.match_and_annotate(frame, self.left_arrow, (255, 0, 0), "Left"):
            return True, "Left"

        return False, None

    def detect_contours(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return len(contours) > 0

    def match_and_annotate(self, frame, template, color, label):
        if template is None:
            return False

        result = cv2.matchTemplate(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        if max_val > 0.7:  # Matching threshold
            top_left = max_loc
            h, w = template.shape
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv2.rectangle(frame, top_left, bottom_right, color, 2)
            cv2.putText(frame, label, (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            return True
        return False

class ArrowDetected(Node):
    def __init__(self, arrow_detector):
        self.arrow_detector = arrow_detector

    def run(self):
        rospy.loginfo("Checking if arrow is detected...")
        detected, direction = self.arrow_detector.process_frame()
        if detected:
            rospy.loginfo(f"Arrow detected: {direction}")
            self.arrow_detector.detected = True
            self.arrow_detector.direction = direction
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

class GetArrowDirection(Node):
    def __init__(self, arrow_detector):
        self.arrow_detector = arrow_detector

    def run(self):
        rospy.loginfo("Getting arrow direction...")
        if self.arrow_detector.detected and self.arrow_detector.direction:
            rospy.loginfo(f"Arrow direction: {self.arrow_detector.direction}")
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

class MoveTowardsArrow(Node):
    def __init__(self):
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        #rospy.sleep(0.1)  # Allow time for publisher connection

    def run(self):
        rospy.loginfo("Moving straight...")
        twist = Twist()
        twist.linear.x = 2.0
        twist.angular.z = 0  # Explicitly set angular velocity to zero
        duration = 5  # Move straight for 10 seconds
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            rospy.sleep(0.4)
        twist.linear.x = 0
        twist.angular.z = 0  # Stop movement
        self.publisher.publish(twist)
        return NodeStatus.SUCCESS


        
class MoveLeft(Node):
    def __init__(self):
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)  # Allow time for publisher connection

    def run(self):
        rospy.loginfo("Rotating left 90 degrees...")
        twist = Twist()
        twist.angular.z = 1.0  # Positive angular velocity for counter-clockwise rotation
        duration = 6.28  # Time for 90 degrees at 0.5 rad/s
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            rospy.sleep(3.5)
        twist.angular.z = 0  # Stop rotation
        self.publisher.publish(twist)
        return NodeStatus.SUCCESS

class MoveRight(Node):
    def __init__(self):
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)  # Allow time for publisher connection

    def run(self):
        rospy.loginfo("Rotating right 90 degrees...")
        twist = Twist()
        twist.angular.z = -1.0  # Negative angular velocity for clockwise rotation
        duration = 6.28  # Time for 90 degrees at 0.5 rad/s
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            rospy.sleep(3.5)
        twist.angular.z = 0  # Stop rotation
        self.publisher.publish(twist)
        return NodeStatus.SUCCESS


# Main Function
if __name__ == "__main__":
    rospy.init_node('robot_behavior_tree', anonymous=True)

    # Initialize arrow detection
    #arrow_detector = ArrowDetection()

    # Build behavior tree
    behavior_tree = SequenceNode([
        MoveTowardsArrow(), MoveLeft() , MoveTowardsArrow() , MoveRight(), MoveTowardsArrow()
    ])

    rate = rospy.Rate(10)  # 10Hz loop rate
    #while not rospy.is_shutdown():
    status = behavior_tree.run()
    rospy.loginfo(f"Behavior Tree Status: {status}")
    rate.sleep()

