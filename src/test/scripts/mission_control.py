
    #     # straight : 100 per 1 line
    #     self.mission_list = [
    #         {"type": "straight", "duration": 300},
    #         {"type": "straight", "duration": 100},
    #         {"type": "straight", "duration": 100},
    #         {"type": "corner", "duration": 150},
    #         {"type": "straight", "duration": 100},
    #         {"type": "straight", "duration": 100},
    #         {"type": "corner", "duration": 150},
    #         {"type": "straight", "duration": 100},
    #         {"type": "corner", "duration": 150}
    #     ]
    #     self.current_mission_idx = 0
    #     self.current_frame_count = 0

    # def determineMission(self):
    #     if self.mission_list and self.current_mission_idx < len(self.mission_list):
    #         mission = self.mission_list[self.current_mission_idx]
    #         if self.current_frame_count >= mission["duration"]:
    #             self.current_frame_count = 0
    #             self.current_mission_idx += 1
    #             print(mission)
    #             return mission
    #     return None
