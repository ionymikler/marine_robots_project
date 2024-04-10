#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import tf
import math

class OdomQuatToEuler:
    def __init__(self):
        rospy.init_node('odom_quat_to_euler', anonymous=True)
        
        self.odom_sub = rospy.Subscriber('/bluerov2/pose_gt', Odometry, self.odom_callback)
        self.euler_angle_pub = rospy.Publisher('/bluerov2/euler_angle_gt', Point, queue_size=10)
        

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        euler_msg = Point()
        euler_msg.x = roll
        euler_msg.y = pitch
        euler_msg.z = yaw

        self.euler_angle_pub.publish(euler_msg)

if __name__ == '__main__':
    try:
        odom_quat_to_euler = OdomQuatToEuler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
