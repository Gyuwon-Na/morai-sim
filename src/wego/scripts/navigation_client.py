#! /usr/bin/env python3

import rospy
import subprocess

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib

class NavigationClient():
    def __init__ (self):
        self.client=actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.goal_list = list()

        self.waypoint_1 = MoveBaseGoal ()
        self.waypoint_1.target_pose.header.frame_id='map'
        self.waypoint_1.target_pose.pose.position.x = 26.064017579577122 #-1.1940211649725783
        self.waypoint_1.target_pose.pose.position.y = -16.74313145004971 #5.890244895651946
        self.waypoint_1.target_pose.pose.orientation.w = 0.999987252700383 #0.9855033486947348
        self.waypoint_1.target_pose.pose.orientation.z = 0.00504920159433932 #-0.16965597458228196

        self.goal_list.append(self.waypoint_1)

        self.waypoint_2 = MoveBaseGoal()
        self.waypoint_2.target_pose.header.frame_id='map'
        self.waypoint_2.target_pose.pose.position.x = 46.3362157208237 #15.237531224023153            #7.182380030530805
        self.waypoint_2.target_pose.pose.position.y = -26.712699950893015 #-9.704318287495006            #-7.0002156353531415
        self.waypoint_2.target_pose.pose.orientation.w = 0.9999993944636794 #0.9818604841189263         #0.5802978498711643
        self.waypoint_2.target_pose.pose.orientation.z = 0.0011004872895951756 #-0.18960482516472962       #-0.814404325525659

        self.goal_list.append(self.waypoint_2)

        self.sequence = 0
        self.start_time = rospy.Time.now()

    def run(self):
        if self.client.get_state() != GoalStatus.ACTIVE:
            self.start_time = rospy.Time.now()
            self.sequence = (self.sequence+1)%2
            self.client.send_goal(self.goal_list[self.sequence])
        else:
            if (rospy.Time.now().to_sec() - self.start_time.to_sec()) > 30.0:
                self.stop()


    def stop(self):
        self.client.cancel_all_goals()


    def execute_main_7(self):
        command = ["python3", "/home/ahs/main_ws/src/test/scripts/Main_7.py"]
        subprocess.run(command)

def main():
    rospy.init_node('navigation_client')
    nc =NavigationClient()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        nc.run()
        rate.sleep()

if __name__ == '__main__':
    main()