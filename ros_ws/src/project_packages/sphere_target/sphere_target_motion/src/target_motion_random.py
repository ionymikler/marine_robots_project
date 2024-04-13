#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
import random

def move_publisher():
    rospy.init_node('target_motion_line', anonymous=True)
    pub = rospy.Publisher('sphere_target/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    t_move = 10.0
    v_move = 0.5
    turnrate = 3.14/4

    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = v_move  # Initial linear velocity
        twist_msg.angular.z = 0

        # Publish initial velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < random.random()*t_move:
            pub.publish(twist_msg)
            rate.sleep()

        # Change linear velocity to -speed
        twist_msg.linear.x = 0
        twist_msg.angular.z = turnrate

        # Publish negative velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < random.random()*8:
            pub.publish(twist_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        move_publisher()
    except rospy.ROSInterruptException:
        pass