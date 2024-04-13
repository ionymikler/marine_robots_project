#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def move_publisher():
    rospy.init_node('target_motion_line', anonymous=True)
    pub = rospy.Publisher('sphere_target/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    t_move = 10.0
    v_move = 0.5

    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = v_move  # Initial linear velocity

        # Publish initial velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < t_move:
            pub.publish(twist_msg)
            rospy.loginfo("Published Twist()")
            rate.sleep()

        # Change linear velocity to -speed
        twist_msg.linear.x = -v_move

        # Publish negative velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < t_move:
            pub.publish(twist_msg)
            rospy.loginfo("Published Twist()")
            rate.sleep()

if __name__ == '__main__':
    try:
        move_publisher()
    except rospy.ROSInterruptException:
        pass