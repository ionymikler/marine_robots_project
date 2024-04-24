#!/usr/bin/env python

import time
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class PIDController():

    def __init__(self, gains) -> None:
        self.gains = gains
        self.pv = 0
        self.sp = 0
        self.error_sum = 0
        self.error_prev = 0
        self.last_time = time.time()

    def calcPID(self, sp, pv):
        self.pv = pv
        self.sp = sp

        error = self.sp - self.pv
        
        current_time = time.time()
        dt = current_time - self.last_time

        self.error_sum += error * dt

        self.p_term = self.gains[0] * error
        self.i_term = self.gains[1] * self.error_sum
        self.d_term = self.gains[2] * ((error - self.error_prev) / dt)

        max_i_term = 1
        self.i_term = max(min(self.i_term, max_i_term), -max_i_term)

        self.error_prev = error
        self.last_time = current_time

        control_input = self.p_term + self.i_term + self.d_term

        return control_input

class ControlNode():
    def __init__(self) -> None:
        self.pv_z = 0
        self.pv_y = 0
        self.pv_dist = 0

        rospy.init_node('pose_control_node', anonymous=True)

        self.PID_z = PIDController([0.002, 0, 0])
        #self.PID_y = PIDController([0.004, 0, 0.0001])
        self.PID_y = PIDController([0.002, 0, 0])
        self.PID_dist = PIDController([2, 0, 0])

        rospy.Subscriber('/vertical_error', Float32, self.error_callback_z)
        rospy.Subscriber('/horizontal_error', Float32, self.error_callback_y)
        rospy.Subscriber('/x_dist', Float32, self.error_callback_dist)

        self.pub = rospy.Publisher('bluerov2/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(50)

        rospy.loginfo("Control Node Initialized")
    
    def error_callback_z(self, data):
        rospy.loginfo_once("Z error received")
        self.pv_z = data.data

    def error_callback_y(self, data):
        self.pv_y = data.data
        rospy.loginfo_once("Y error received")

    def error_callback_dist(self, data):
        self.pv_dist = data.data
        rospy.loginfo_once("Distance error received")
    
    def control_publisher(self):
        while not rospy.is_shutdown():
            twist_msg = Twist()

            twist_msg.linear.z = self.PID_z.calcPID(0, self.pv_z)
            twist_msg.angular.z = self.PID_y.calcPID(0, self.pv_y)
            twist_msg.linear.x = -(self.PID_dist.calcPID(1, self.pv_dist))
            self.pub.publish(twist_msg)

            self.rate.sleep()

if __name__ == '__main__':
    node = ControlNode()
    try:
        node.control_publisher()
    except rospy.ROSInterruptException:
        pass