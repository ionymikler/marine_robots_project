#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from scipy.spatial.transform import Rotation as R
import numpy as np
import time
import cv2 as cv
from cv_bridge import CvBridge

class SpherePose():
    def __init__(self) -> None:
        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0

    def update_pose(self, img):
        img, px_y, px_z, diam = self.detect_circles(img)

        return img, self.x_pos, self.y_pos, self.z_pos


    def detect_circles(self, img, plot_circles=False):
        px_y, px_z, diam = 0,0,0


        return img, px_y, px_z, diam

class TrackingNode():
    def __init__(self) -> None:
        rospy.init_node('sphere_target_tracking', anonymous=True)
        self.pub = rospy.Publisher('sphere_target/sphere_pos', Pose, queue_size=10)
        rospy.Subscriber("/bluerov2/camera_front/camera_image", Image, self.image_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.cv_bridge = CvBridge()
        self.sphere_detector = SpherePose()

        self.img = Image()
        self.img_circles = np.array((1080,1920))

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0

        self.focal_length = 2.97 # unit is mm
        self.px_width = 1920
        self.px_height = 1080

    def image_callback(self, msg):
        rospy.loginfo_once("Received: Image")
        self.img = msg

    def update_pose(self):
        img = self.cv_bridge.imgmsg_to_cv2(self.img, desired_encoding="passthrough")
        img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)

        img_circles, self.x_pos, self.y_pos, self.z_pos = self.sphere_detector.update_pose(img)

    def publish_pose(self):
        while not rospy.is_shutdown():
            pose_msg = Pose()

    def publish_img(self):
        pass


if __name__ == '__main__':
    node = TrackingNode()
    try:
        node.publish_move()
    except rospy.ROSInterruptException:
        pass