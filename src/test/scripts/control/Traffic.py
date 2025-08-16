#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import morai_msgs.msg as morai_msgs


class Traffic:
    def __init__(self):
        rospy.Subscriber("/GetTrafficLightStatus", morai_msgs.GetTrafficLightStatus, self.traffic_CB)
        self.traffic_msg = morai_msgs.GetTrafficLightStatus()
        self.traffic_flag = 0
        self.prev_signal = 0
        self.traffic_signal = None

    def traffic_CB(self, msg):
        self.traffic_msg = msg
        if self.traffic_msg.trafficLightIndex == "SN000005":
            self.traffic_signal = msg.trafficLightStatus
            # print(self.traffic_signal)
            
            if self.traffic_signal != self.prev_signal:
                self.prev_signal = self.traffic_signal
                self.traffic_flag = 0

            self.traffic_flag += 1