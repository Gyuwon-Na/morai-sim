#!/usr/bin/env python3
#-*- coding:utf-8

import rospy
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import Float64
from math import *
import os 
import numpy as np

class Obstacle:
    def __init__(self):
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_CB)
        self.scan_msg = LaserScan()
        self.obstacle_degrees = []
        self.obstacle_index = []
        self.detected = False
        
    def lidar_CB(self, msg):
        os.system("clear")
        self.scan_msg = msg 
        degree_min = self.scan_msg.angle_min * 180/pi
        degree_angle_increment = self.scan_msg.angle_increment * 180/pi 
        degrees = [degree_min + degree_angle_increment * index for index, value in enumerate(self.scan_msg.ranges)]

        for index, value in enumerate(self.scan_msg.ranges):
            if abs(degrees[index]) < 90.1 and 0 < value < 2:
                print(f"Obstacle detected at {degrees[index]:.2f} degrees, distance: {value:.2f} m")
                self.obstacle_degrees.append(degrees[index])
                self.obstacle_index.append(index)
                self.detected = True
            else:
                self.detected = False

    def avoidance(self):
        try:
            right_space = self.obstacle_index[0] - 180
            left_space = 542 - self.obstacle_index[-1]

            if right_space > left_space:
                right_index_avg = (self.obstacle_degrees[self.obstacle_index[0]] - 90) / 2
                degree_avg = right_index_avg
            else:
                left_index_avg = (self.obstacle_degrees[self.obstacle_index[-1]] + 90) / 2
                degree_avg = left_index_avg

        except IndexError:
            degree_avg = 0
        
        return degree_avg / 90 + 0.5