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

        # camera parameters: https://bluerobotics.com/store/sensors-cameras/cameras/cam-usb-low-light-r1/

        self.focal_length = 2.97/0.0028 # unit is px (f=2.97mm, px=2.8um)
        self.px_width_base = 1920
        self.px_height_base = 1080

        self.sphere_size = 0.5

    def update_pose(self, img):
        circles= self.detect_circles(img)
        if circles.shape[0] > 0:
            px_width, px_height, radius = circles[0,0]
            px_width = px_width - (self.px_width_base/2)
            px_height = px_height - (self.px_height_base/2)
            diameter = 2*radius

        self.x_pos = self.update_x_pos(diameter)
        self.y_pos = self.update_y_pos(px_width)
        self.z_pos = self.update_z_pos(px_height)

        return self.x_pos, self.y_pos, self.z_pos

    def update_x_pos(self, size):
        return ((self.focal_length)*self.sphere_size)/size
    
    def update_y_pos(self, px_width):
        return self.x_pos*px_width/(self.focal_length)

    def update_z_pos(self, px_height):
        return self.x_pos*px_height/(self.focal_length)

    def detect_circles(self, img, plot_circles=False):
        img_gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        # Reduce the noise to avoid false circle detection
        img_gray = cv.medianBlur(img_gray, 5)

        rows = img_gray.shape[0]
        circles = cv.HoughCircles(image=img_gray, method=cv.HOUGH_GRADIENT,dp= 1,
                                    minDist= rows / 8,
                                    param1=100, param2=30,
                                    minRadius=1, maxRadius=int(0.4*rows)
                                    )
        
        return np.array([]) if circles is None else circles