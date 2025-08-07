#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import sleep

class MissionControl:
    def __init__(self):
        self.current_mission_idx = 0
        self.current_frame_count = 0

        self.mission_list = [
            {"type": "straight", "duration": 180},
            {"type": "curve", "duration": 30},
            {"type": "straight", "duration": 120},
            {"type": "curve", "duration": 30},
            {"type": "straight", "duration": 60},
            {"type": "curve", "duration": 30},
            {"type": "straight", "duration": 60},
            {"type": "curve", "duration": 30},
            {"type": "straight", "duration": 60},
            {"type": "curve", "duration": 30},
            {"type": "straight", "duration": 80},
            {"type": "curve", "duration": 30},
            {"type": "straight", "duration": 160},
            {"type": "curve", "duration": 30},
            {"type": "straight", "duration": 80}
        ]
        self.current_mission_idx = 0
        self.current_frame_count = 0

    def determineMission(self):
        if self.mission_list and self.current_mission_idx < len(self.mission_list):
            mission = self.mission_list[self.current_mission_idx]

            if self.current_frame_count >= mission["duration"]:
                self.current_frame_count = 0
                self.current_mission_idx += 1

                if self.current_mission_idx < len(self.mission_list):
                    rospy.loginfo(f"[Mission] Next: {self.mission_list[self.current_mission_idx]}")
                else:
                    rospy.loginfo("[Mission] All missions completed.")
            else:
                self.current_frame_count += 1

            if self.current_mission_idx < len(self.mission_list):
                return self.mission_list[self.current_mission_idx]

        return None

if __name__ == "__main__":
    rospy.init_node("mission_test_node")
    mc = MissionControl()

    rate = rospy.Rate(10)  # 10 Hz (0.1초마다 호출)
    while not rospy.is_shutdown():
        current_mission = mc.determineMission()
        if current_mission:
            rospy.loginfo(f"Current Mission: {current_mission['duration'] - mc.current_frame_count} frames remaining")
        else:
            rospy.loginfo("No more missions.")
            break
        rate.sleep()
