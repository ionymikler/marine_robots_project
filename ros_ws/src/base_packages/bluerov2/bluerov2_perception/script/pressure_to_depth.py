#!/usr/bin/env python

import rospy
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
import numpy as np

class DepthEstimator:
    def __init__(self, standard_pressure, kpa_per_m):
        self.standard_pressure = standard_pressure
        self.frame_id = "bluerov2/base_link"
        self.kpa_per_m = kpa_per_m
        self.depth_values = []
        self.depth_rate_values = []
        self.time_values = []
        self.depth_publisher = rospy.Publisher("bluerov2/pressure_depth", PoseWithCovarianceStamped, queue_size=10)
        self.depth_rate_publisher = rospy.Publisher("bluerov2/pressure_depth_rate", TwistWithCovarianceStamped, queue_size=10)
        self.pressure_subscriber = rospy.Subscriber("bluerov2/pressure", FluidPressure, self.pressure_callback)

    def pressure_callback(self, data):
        print("Get raw data",data.fluid_pressure)
        depth = (data.fluid_pressure - self.standard_pressure) / (self.kpa_per_m * 1000.0)
        depth = -depth*1000 # in meter
        self.depth_values.append(depth)
        self.time_values.append(rospy.Time.now())
        
        if len(self.depth_values) > 50:  # Keep only the last 50 depth values for better estimation
            self.depth_values.pop(0)
            self.time_values.pop(0)
        
        depth_rate = self.calculate_depth_rate()
        depth_variance = self.calculate_variance(self.depth_values)
        depth_rate_variance = self.calculate_variance(self.depth_rate_values)
        
        self.publish_depth(depth, depth_variance)
        self.publish_depth_rate(depth_rate, depth_rate_variance)

    def calculate_variance(self, depths):
        std_dev = np.std(depths)
        variance = std_dev ** 2
        return variance

    def calculate_depth_rate(self):
        if len(self.depth_values) < 2:
            return 0.0  # Return 0 if there's not enough data to calculate rate
        
        depth_change = self.depth_values[-1] - self.depth_values[-2]
        time_change = (self.time_values[-1] - self.time_values[-2]).to_sec()
        depth_rate = depth_change / time_change
        
        self.depth_rate_values.append(depth_rate)

        return depth_rate

    def publish_depth(self, depth, variance):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.z = depth

        pose_with_covariance = PoseWithCovarianceStamped()
        pose_with_covariance.header = pose.header
        pose_with_covariance.pose.pose = pose.pose

        # Set covariance matrix
        covariance = np.zeros((6,6), dtype=float)
        covariance[2,2] = variance 

        pose_with_covariance.pose.covariance = covariance.flatten().tolist()  

        self.depth_publisher.publish(pose_with_covariance)

    def publish_depth_rate(self, depth_rate, variance):
        twist = TwistWithCovarianceStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = self.frame_id
        twist.twist.twist.linear.z = depth_rate
        
        # Set covariance matrix for depth rate
        covariance = np.zeros((6, 6), dtype=float)
        covariance[2, 2] = variance  # Assuming variance of depth rate
        
        twist.twist.covariance = covariance.flatten().tolist()

        self.depth_rate_publisher.publish(twist)


def main():
    rospy.init_node('depth_estimator', anonymous=True)
    standard_pressure = 101.325  # kPa
    kpa_per_m = 9.80638  # kPa/m
    depth_estimator = DepthEstimator(standard_pressure, kpa_per_m)
    rospy.spin()

if __name__ == '__main__':
    main()
