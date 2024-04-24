#!/usr/bin/env python
# Made by: Jonathan Mikler
# Creation date: 2024-04-24

import numpy as np
import ros_numpy
from sensor_msgs.msg import LaserScan, PointCloud2

import laser_geometry.laser_geometry as lg

class SonarProcessor():
    def __init__(self, x_min, x_max, y_min, y_max) -> None:
        self.laser_projector = lg.LaserProjection()

        self.x_min:float = x_min
        self.x_max:float = x_max
        self.y_min:float = y_min
        self.y_max:float = y_max

    def process_sonar_data(self, sonar_msg:LaserScan) -> PointCloud2:
        """
        Process the sonar data and return the distance to the target.
        :param sonar_data: The sonar data.
        :return: The distance to the target.
        """
        # 1. convert 2 pointcloud
        # 2. define in-zone and filter out-of-zone
        # 3. get 3D mean and std of in-zone
        _pc2_msg = self.laser_projector.projectLaser(sonar_msg)
        _oc_np:np.ndarray = ros_numpy.numpify(_pc2_msg) # x,y,z,intensity
        oc = np.array([[p[0], p[1]] for p in _oc_np])

        # filter all points that are not in the zone
        oc = oc[(oc[:,0] > self.x_min) & (oc[:,0] < self.x_max) & (oc[:,1] > self.y_min) & (oc[:,1] < self.y_max)]
        print(oc.mean())
        # NOTE: Processing this further is a lot of work. stopping for now
        print(oc[-1])
        

        return _pc2_msg

