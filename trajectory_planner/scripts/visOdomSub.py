#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry

def odometryCb(msg):
    print (msg.pose.pose.position.x, ",", msg.pose.pose.position.y, ",", msg.pose.pose.position.z, ",", msg.pose.pose.orientation.w, ",", msg.pose.pose.orientation.x, ",", msg.pose.pose.orientation.y, ",", msg.pose.pose.orientation.z)

if __name__ == "__main__":
    rospy.init_node('oodometry', anonymous=True) #make node 
    rospy.Subscriber('/rtabmap/odom',Odometry,odometryCb)
    rospy.spin()
