#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

class MoveNode():
    def __init__(self) -> None:
        rospy.init_node('target_motion_square', anonymous=True)
        self.pub = rospy.Publisher('sphere_target/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('sphere_target/pose_gt', Odometry, self.callback)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.t_move = 10.0
        self.v_move = 0.5
        self.turnrate = 3.14/4
        self.position = Odometry()

    def callback(self, msg):
        self.position = msg

    def publish_move(self):
        while not rospy.is_shutdown():
            twist_msg = Twist()
            twist_msg.linear.x = self.v_move  # Initial linear velocity
            twist_msg.angular.z = 0

            # Publish initial velocity for T/2 seconds
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < self.t_move:
                self.pub.publish(twist_msg)
                rospy.loginfo("Published Twist()")
                self.rate.sleep()

            # Start turning
            twist_msg.linear.x = 0
            twist_msg.angular.z = self.turnrate

            # Publish turnrate for approximately one 90° turn
            r = R.from_quat([self.position.pose.pose.orientation.x, 
                             self.position.pose.pose.orientation.y, 
                             self.position.pose.pose.orientation.z, 
                             self.position.pose.pose.orientation.w])
            pose_init = r.as_euler('xyz', degrees=True)[2]
            pose_turned = pose_init
            while (abs(pose_turned - pose_init)) <= 90.0:
                self.pub.publish(twist_msg)
                rospy.loginfo("Published Twist()")
                r = R.from_quat([self.position.pose.pose.orientation.x, 
                             self.position.pose.pose.orientation.y, 
                             self.position.pose.pose.orientation.z, 
                             self.position.pose.pose.orientation.w])
                pose_turned = r.as_euler('xyz', degrees=True)[2]
                self.rate.sleep()   

if __name__ == '__main__':
    node = MoveNode()
    try:
        node.publish_move()
    except rospy.ROSInterruptException:
        pass