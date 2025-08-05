#! /usr/bin/env python3

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

from turtlesim.msg import Pose
import tf_conversions

from turtlesim.srv import Spawn, SpawnRequest

class TurtleTwoBroadcaster:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.transformStamped = TransformStamped()
        self.transformStamped.header.frame_id = "world"
        self.transformStamped.child_frame_id = "turtle2"

        service =rospy.ServiceProxy('spawn',Spawn)
        service.wait_for_service()
        srv = SpawnRequest()
        srv.name = 'turtle2'
        service(srv)

        rospy.Subscriber('/turtle2/pose', Pose, self.callback)

    def callback(self, data):
        self.transformStamped.transform.translation.x = data.x
        self.transformStamped.transform.translation.y = data.y

        q=tf_conversions.transformations.quaternion_from_euler(0, 0, data.theta)

        self.transformStamped.transform.rotation.x = q[0]
        self.transformStamped.transform.rotation.y = q[1]
        self.transformStamped.transform.rotation.z = q[2]
        self.transformStamped.transform.rotation.w = q[3]

    def broadcast(self):
        self.transformStamped.header.stamp=rospy.Time().now()
        self.br.sendTransform(self.transformStamped)

def main():
    rospy.init_node('turtle_Two_broadcaster')
    ttb = TurtleTwoBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        ttb.broadcast()

if __name__ == "__main__":
    main()