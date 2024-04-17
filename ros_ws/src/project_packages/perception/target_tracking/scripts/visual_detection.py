#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from typing import Tuple

from target_tracking.image_utils import img_error, \
    paint_centerlines_in_image, paint_x_in_image, paint_img_errors, paint_circles_in_image

from target_tracking.target_utils import find_targets

class TargetDetectorNode:
    def __init__(self):
        rospy.init_node("target_detector")
        rospy.loginfo("Starting TargetDetectorNode as target_detector.")

        self.cv_bridge = CvBridge()

        self.start_ros_interfaces()
    
    def start_ros_interfaces(self):
        self.image_sub = rospy.Subscriber("/bluerov2/camera_front/camera_image", Image, self.image_callback)
        self.horizontal_error_pub = rospy.Publisher("/horizontal_error", Float32, queue_size=1)
        self.vertical_error_pub = rospy.Publisher("/vertical_error", Float32, queue_size=1)
        self.processed_image_pub = rospy.Publisher("/processed_image", Image, queue_size=1)
    
    def image_callback(self, image_msg: Image):
        rospy.loginfo_once("Received first image message.")
        image_og = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        rgb = cv.cvtColor(image_og, cv.COLOR_BGR2RGB)

        targets, error_h, error_v = self.find_target(rgb)
        rospy.loginfo(f"Errors: h={error_h}, v={error_v}")
        # Publish errors
        self.horizontal_error_pub.publish(error_h)
        self.vertical_error_pub.publish(error_v)

        self.post_process_image(rgb, targets)

    def post_process_image(self, image: np.ndarray, targets:np.ndarray)->np.ndarray:
        # Publish processed image for visual validation
        processed_img = paint_circles_in_image(image, targets, n=3)
        processed_img = paint_centerlines_in_image(processed_img)
        processed_img = paint_x_in_image(processed_img, image.shape[1]//2, image.shape[0]//2)

        target_center_uv = (int(targets[0,0,0]), int(targets[0,0,1]))
        error_h, error_v = img_error(target_center_uv, (image.shape[1], image.shape[0]))

        processed_img = paint_img_errors(processed_img, target_center_uv, error_h, error_v)

        self.processed_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(processed_img, encoding="passthrough"))
    
    def find_target(self, image: np.ndarray)->Tuple[np.ndarray, float, float]:
        """
        Detects the targets in the image and returns the error in horizontal and vertical directions
        -
        image: np.ndarray: RGB image
        """
        img_dims = (image.shape[1], image.shape[0])

        targets = find_targets(image)
        rospy.loginfo_throttle(0.5,f"Targets detected: {targets.shape}")

        target_1 = targets[0,0]
        center_x, center_y, _ = target_1 # x,y,radius for a circle target
        center_u, center_v = int(center_x), int(center_y) # u,v to pixel indices, not great
        error_h, error_v = img_error((center_u, center_v), img_dims)

        return targets, error_h, error_v


if __name__ == "__main__":
    target_detector = TargetDetectorNode()
    rospy.spin()