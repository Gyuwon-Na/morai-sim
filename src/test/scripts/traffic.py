#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import morai_msgs.msg as morai_msgs


class Traffic_Sub:
    def __init__(self):
        rospy.Subscriber("/GetTrafficLightStatus", morai_msgs.GetTrafficLightStatus, self.traffic_CB)
        self.traffic_msg = morai_msgs.GetTrafficLightStatus()
        self.traffic_flag = 0
        self.prev_signal = 0

    def traffic_CB(self, msg):
        self.traffic_msg = msg
        self.get_traffic_status(msg)

    def get_traffic_status(self, msg):
        signal = msg.trafficLightStatus

        if signal == 1:
            pass
            # print(f"{msg.trafficLightIndex}: red")
        elif signal == 4:
            pass
            # print(f"{msg.trafficLightIndex}: yellow")
        elif signal == 16 or signal == 33:
            pass
            # print(f"{msg.trafficLightIndex}: green")