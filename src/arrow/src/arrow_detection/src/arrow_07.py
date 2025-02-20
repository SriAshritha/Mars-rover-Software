#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32  # Importing Int32 message type for publishing 0 or 1
from std_msgs.msg import Int32  # Importing Float32 for publishing distance
from cv_bridge import CvBridge, CvBridgeError

# Initialize ROS node
rospy.init_node('arrow_detection_node', anonymous=True)

# Initialize the CvBridge class
bridge = CvBridge()

# Create ROS Publishers
direction_pub = rospy.Publisher('/arrow_direction', Int32, queue_size=10)
distance_pub = rospy.Publisher('/arrow_distance', Int32, queue_size=10)

# Define Area of Interest (AOI)
AOI_X_START = 0.3  # Narrowed from 0.2 for a smaller AOI
AOI_X_END = 0.7    # Narrowed from 0.8
AOI_Y_START = 0.3  # Narrowed from 0.2
AOI_Y_END = 0.7    # Narrowed from 0.8

# Webcam Specifications
RESOLUTION_WIDTH = 1280  # Horizontal resolution of 720p
RESOLUTION_HEIGHT = 720  # Vertical resolution of 720p
DIAGONAL_FOV = 55  # Diagonal Field of View in degrees
REAL_ARROW_WIDTH = 17.0  # Real-world arrow width in cm

# Distance scaling factor (calibrated to correct measurement errors)
SCALING_FACTOR = 1.67

# Compute horizontal FoV (hFoV) and focal length
aspect_ratio = 16 / 9
hFoV = 2 * math.degrees(
    math.atan((16 / math.sqrt(16**2 + 9**2)) * math.tan(math.radians(DIAGONAL_FOV / 2)))
)
focal_length = RESOLUTION_WIDTH / (2 * math.tan(math.radians(hFoV / 2)))

def is_arrow_shape(contour, approx):
    """
    Check if a contour has the shape of an arrow.
    """
    if len(approx) < 7 or len(approx) > 10:  # Adjust for tighter vertex count
        return False

    # Compute the bounding rectangle
    x, y, w, h = cv2.boundingRect(approx)
    aspect_ratio = w / float(h)

    if not (1.5 < aspect_ratio < 3.5):  # Arrows typically have this aspect ratio
        return False

    # Calculate centroid
    M = cv2.moments(contour)
    if M['m00'] == 0:
        return False
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    # Verify the arrow tip (farthest point from the centroid)
    vertices = approx.reshape(-1, 2)
    tip = max(vertices, key=lambda point: np.linalg.norm(point - np.array([cx, cy])))

    # Check if the tip is significantly farther than other vertices
    tip_distance = np.linalg.norm(tip - np.array([cx, cy]))
    avg_distance = np.mean([np.linalg.norm(v - np.array([cx, cy])) for v in vertices])
    
    if tip_distance < 1.5 * avg_distance:  # The tip must be a prominent outlier
        return False

    return True

def detect_arrow_direction(contour, approx):
    """
    Determine the direction of the arrow.
    Publish 0 for Left, 1 for Right.
    """
    vertices = approx.reshape(-1, 2)

    # Calculate the centroid
    M = cv2.moments(contour)
    if M['m00'] == 0:
        return 0  # Default to 0 if the direction is unknown
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    # Find the tip of the arrow
    tip = max(vertices, key=lambda point: np.linalg.norm(point - np.array([cx, cy])))

    # Determine direction based on tip's x-coordinate relative to centroid
    if tip[0] > cx:
        return 0  # Left
    else:
        return 1  # Right

def calculate_distance(pixel_width):
    """
    Calculate the corrected distance using the scaling factor.
    """
    if pixel_width == 0:
        return None  # Avoid division by zero
    measured_distance = (REAL_ARROW_WIDTH * focal_length) / pixel_width
    corrected_distance = measured_distance * SCALING_FACTOR
    return corrected_distance

def image_callback(msg):
    try:
        # Convert the ROS Image message to OpenCV format
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Resize frame for consistent processing
    frame = cv2.resize(frame, (RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
    height, width = frame.shape[:2]

    # Define AOI coordinates
    x_start = int(AOI_X_START * width)
    x_end = int(AOI_X_END * width)
    y_start = int(AOI_Y_START * height)
    y_end = int(AOI_Y_END * height)

    # Draw AOI for visualization
    cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (255, 255, 0), 2)

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Use Canny edge detection with adjusted thresholds
    edges = cv2.Canny(blurred, 30, 200)

    # Perform dilation to connect fragmented edges
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dilated = cv2.dilate(edges, kernel, iterations=1)

    # Find contours with hierarchical retrieval
    contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    arrow_detected = False

    for contour in contours:
        if cv2.contourArea(contour) < 1000:  # Adjust contour area threshold
            continue

        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        x, y, w, h = cv2.boundingRect(approx)
        if not (x_start <= x and x + w <= x_end and y_start <= y and y + h <= y_end):
            continue

        if is_arrow_shape(contour, approx):
            direction = detect_arrow_direction(contour, approx)
            distance = calculate_distance(w)

            # Publish direction and distance
            direction_pub.publish(direction)
            if distance:
                distance_pub.publish(distance)

            # Display direction and distance
            distance_text = f"Distance: {distance:.2f} cm" if distance else "Distance: Unknown"
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, f"Arrow: {'Left' if direction == 0 else 'Right'}", (x, y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, distance_text, (x, y - 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 255), 2)
            arrow_detected = True
            break

    if not arrow_detected:
        cv2.putText(frame, "No Arrow Detected", (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 255), 2)

    cv2.imshow("Webcam Feed", frame)
    cv2.imshow("Edges", dilated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User requested shutdown.")

# Subscribe to the camera topic
rospy.Subscriber("/dev/video1", Image, image_callback)

# Keep the program running until the user shuts it down
rospy.spin()

# Release resources
cv2.destroyAllWindows()

