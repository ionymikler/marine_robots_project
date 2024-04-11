#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# from target_tracking.detections import detect_circles, paint_circles_in_image

def detect_circles(image: np.ndarray)->np.ndarray:
    rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)

    gray = cv.cvtColor(rgb, cv.COLOR_RGB2GRAY)
    # Reduce the noise to avoid false circle detection
    gray = cv.medianBlur(gray, 5)

    rows = gray.shape[0]
    circles = cv.HoughCircles(image=gray, method=cv.HOUGH_GRADIENT,dp= 1,
                                minDist= rows / 8,
                                param1=100, param2=30,
                                minRadius=1, maxRadius=int(0.8*rows)
                                )
    
    return np.array([]) if circles is None else circles

def paint_circles_in_image(image: np.ndarray, circles: np.ndarray, n:int=1)->np.ndarray:
    """
    paints the fist n circles from 'circles' in the image
    """
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i, circle_i in enumerate(circles[0, :], start=1):
            if i > n:
                break
            center = (circle_i[0], circle_i[1])
            radius = circle_i[2]

            cv.circle(image, center, 1, (0, 100, 100), 3)
            cv.circle(image, center, radius, (255, 0, 255), 3)
            # write index next to circle center
            cv.putText(image, str(i), (circle_i[0], circle_i[1]), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 255), 5)

    return image

class TargetDetectorNode:
    def __init__(self):
        rospy.init_node("target_detector")
        rospy.loginfo("Starting TargetDetectorNode as target_detector.")

        self.cv_bridge = CvBridge()

        self.start_ros_interfaces()
    
    def start_ros_interfaces(self):
        self.image_sub = rospy.Subscriber("/bluerov2/camera_front/camera_image", Image, self.image_callback)
        self.processed_image_pub = rospy.Publisher("/processed_image", Image, queue_size=1)
    
    def image_callback(self, image_msg: Image):
        rospy.loginfo_once("Received first image message.")
        image_np = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

        processed_image = self.process_image(image_np)

        self.processed_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(processed_image, encoding="passthrough"))
    
    def process_image(self, image: np.ndarray)->np.ndarray:
        rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        # TODO: take this functions to another module. Problem with setup.py
        circles = detect_circles(rgb)
        rospy.loginfo_throttle(0.5,f"circles detected: {circles.shape}")
        rgb = paint_circles_in_image(rgb, circles, n=3)

        return rgb


if __name__ == "__main__":
    target_detector = TargetDetectorNode()
    rospy.spin()