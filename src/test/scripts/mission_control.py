#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

class MissionControl:
    def __init__(self):
        self.current_mission_idx = 0
        self.current_frame_count = 0

        # straight : 100 per 1 line
        self.mission_list = [
            {"type": "straight", "duration": 100},
            {"type": "straight", "duration": 100},
            {"type": "mission", "duration": 100},
            {"type": "straight", "duration": 100},
            {"type": "mission", "duration": 100},
            {"type": "straight", "duration": 100},
            {"type": "curve", "duration": 100},
        ]
        self.current_mission_idx = 0
        self.current_frame_count = 0

    def determineMission(self):
        if self.mission_list and self.current_mission_idx < len(self.mission_list):
            mission = self.mission_list[self.current_mission_idx]

            # duration만큼 프레임이 지나면 다음 미션으로 이동
            if self.current_frame_count >= mission["duration"]:
                self.current_frame_count = 0
                self.current_mission_idx += 1

                if self.current_mission_idx < len(self.mission_list):
                    rospy.loginfo(f"[Mission] Next: {self.mission_list[self.current_mission_idx]}")
                else:
                    rospy.loginfo("[Mission] All missions completed.")
            else:
                self.current_frame_count += 1

            # 항상 현재 미션 리턴
            if self.current_mission_idx < len(self.mission_list):
                return self.mission_list[self.current_mission_idx]

        return None

