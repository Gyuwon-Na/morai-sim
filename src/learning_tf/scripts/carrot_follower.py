#! /usr/bin/env python3

import rospy
import tf2_ros
import math
from geometry_msgs.msg import Twist


class CarrotFollower:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.pub = rospy.Publisher('turtle2/cmd_vel', Twist, queue_size=10)

    def broadcast(self):
        try:
            trans = self.tfBuffer.lookup_transform('turtle2', 'carrot', rospy.Time.now(),timeout=rospy.Duration(0.1))
            msg=Twist()
            msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2 )
            msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            self.pub.publish(msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('except')

def main():
    rospy.init_node('Carrot_follower')
    cf=CarrotFollower()
    rate = rospy.Rate (10)
    while not rospy. is_shutdown () :
        rate.sleep()
        cf.broadcast()
        

if __name__ == "__main__":
    main()