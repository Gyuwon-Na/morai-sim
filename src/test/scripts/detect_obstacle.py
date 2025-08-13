#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import numpy as np
import math
import cv2
from sensor_msgs.msg import LaserScan


class LidarSub:
    def __init__(self):
        rospy.Subscriber("/lidar2D",LaserScan,self.lidar_CB)    
        self.Laser_msg = LaserScan()


    def lidar_CB(self, msg):
        self.scan_msg = msg
        