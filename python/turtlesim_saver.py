#!/usr/bin/env python3
import rospy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import sys


LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

# flag = 1

# class vel_manipulator:

#     def __init__(self):
#         pub_topic_name = "/cmd_vel"
#         sub_topic_name = "/odom"

#         self.pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10)
#         self.number_subscriber = rospy.Subscriber(sub_topic_name, Odometry, self.pose_callback)
#         self.velocity_msg = Twist()

    # def pose_callback(self, msg):
    #     if(msg.pose.pose.position.x >= 1):
    #         ##stopping condition
    #         flag=0
    #         rospy.loginfo("flag turned to zero")
    #         self.velocity_msg.linear.x = 0
    #         msg.pose.pose.position.x = 0
            
    #     else:
    #         self.velocity_msg.linear.x = 0.5

    # self.pub.publish(self.velocity_msg)

if __name__ == '__main__':
    node_name = "turtlesimsaver"
    rospy.init_node(node_name)
    rospy.loginfo("starting the codee now")
    # while flag==1 :
    #     rospy.loginfo("I am inside the while loop where flag is equal to one")
    #     vel_manipulator()
    # rospy.spin()
    pub_topic_name = "/cmd_vel"
    sub_topic_name = "/odom"
    pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10)
    rate = rospy.Rate(2)
    # sub = rospy.Subscriber(sub_topic_name, Odometry, pose_callback)
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x =1 
        pub.publish(msg)
        # rate.sleep()
    
    msgg = Twist()
    msgg.linear.x = 0
    pub.publish(msgg)

    rospy.loginfo("Game over")
        