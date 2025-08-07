#!/usr/bin/env python3
#-*- coding:utf-8

import rospy
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import Float64
from math import *
import os 
import numpy as np
class Turtle_sub:
    def __init__(self):
        rospy.init_node("sim_e_stop")
        self.speed_pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1)
        self.steer_pub = rospy.Publisher("/commands/servo/position",Float64,queue_size=1)
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_CB)
        self.scan_msg = LaserScan()
        self.speed_msg = Float64()
        self.steer_msg = Float64()
        
    def lidar_CB(self,msg):
        os.system("clear")
        self.scan_msg = msg 
        degree_min = self.scan_msg.angle_min * 180/pi
        degree_max = self.scan_msg.angle_max * 180/pi 
        degree_angle_increment = self.scan_msg.angle_increment * 180/pi 
        degrees = [degree_min + degree_angle_increment * index for index, value in enumerate(self.scan_msg.ranges)]
        degree_array = np.array(degrees)

        obstacle_degrees = []
        obstacle_index = []
        
        for index, value in enumerate(self.scan_msg.ranges):
            if abs(degrees[index])<90.1  and 0<value<2:
                obstacle_degrees.append(degrees[index])
                obstacle_index.append(index)
                
            else :
                pass
        
        
        
        try:
            right_space = obstacle_index[0]-180
            left_space = 542-obstacle_index[-1]

            if  right_space > left_space:
                print(f"right:{obstacle_index[0]}")
                right_index_avg = (degrees[obstacle_index[0]]-90) /2
                print(f"right_degree_avg:{right_index_avg}")
                degree_avg = right_index_avg 
                
            else :
                left_index_avg = (degrees[obstacle_index[-1]]+90)/2
                print(f"left:{left_index_avg}")
                print(f"left_degree_avg:{obstacle_index[-1]}")
                degree_avg = left_index_avg 
                
        except :
            degree_avg = 0
            steer = 0.5
        steer = -degree_avg/90
        print(steer)
            
        self.speed_msg.data = 1000
        # self.steer_msg.data = steer    
        self.speed_pub.publish(self.speed_msg)
        # self.steer_pub.publish(self.steer_msg)
def main():
    try:
        turtle_sub = Turtle_sub()   
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()