#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


class vel_manipulator:

    def __init__(self):
        pub_topic_name = "/cmd_vel"
        sub_topic_name = "/odom"

        self.pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10)
        self.number_subscriber = rospy.Subscriber(sub_topic_name, Odometry, self.pose_callback)
        self.velocity_msg = Twist()

    def pose_callback(self, msg):
        if(msg.pose.pose.position.x >= 1):
            self.velocity_msg.linear.x = 0
            msg.pose.pose.position.x = 0
            
        else:
            self.velocity_msg.linear.x = 0.5

        self.pub.publish(self.velocity_msg)

    

if __name__ == '__main__':
    reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_simulation()
    node_name = "turtlesimsaver"
    rospy.init_node(node_name)
    vel_manipulator()
    rospy.spin()
  
            