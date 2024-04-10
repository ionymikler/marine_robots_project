#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, PoseStamped
import tf
import math

class PoseStampedToEuler:
    def __init__(self):
        rospy.init_node('pose_stamped_to_euler', anonymous=True)
        
        self.odom_sub = rospy.Subscriber('/bluerov2/cmd_pose', PoseStamped, self.cmd_pose_callback)
        self.euler_angle_pub = rospy.Publisher('/bluerov2/cmd_euler_angle', Point, queue_size=10)
        

    def cmd_pose_callback(self, msg):
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        euler_msg = Point()
        euler_msg.x = roll
        euler_msg.y = pitch
        euler_msg.z = yaw

        self.euler_angle_pub.publish(euler_msg)

if __name__ == '__main__':
    try:
        pose_stamped_to_euler = PoseStampedToEuler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
