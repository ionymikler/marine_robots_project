#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdometryToPath:
    def __init__(self):
        rospy.init_node('odometry_to_path', anonymous=True)
        
        self.odom_sub = rospy.Subscriber('/bluerov2/pose_gt', Odometry, self.odom_callback)
        self.path_pub = rospy.Publisher('/bluerov2/path_gt', Path, queue_size=10)
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'

    def odom_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        self.path_msg.poses.append(pose_stamped)
        
        # Publish path
        self.path_pub.publish(self.path_msg)

if __name__ == '__main__':
    try:
        odometry_to_path = OdometryToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
