#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import numpy as np
from camera import SlidingWindow
from control import Traffic
from camera import DetectLane
import cv2
from enum import Enum
import time

class Mission(Enum):
    STRAIGHT = 1                  # 일반 차선 추종 주행
    FIRST_CORNER = 2        # 첫 번째 코너 진입
    SECOND_CORNER = 3       # 두 번째 코너 진입
    ROTARY = 4       # 로터리 진입
    TRAFFICLIGHT = 5
    THIRD_CORNER = 6
    FINISH = 7

class AutonomousDriving:
    def __init__(self):
        self.sliding = SlidingWindow.SlidingWindow()
        self.stop_lane_detector = DetectLane.StopLane()
        self.yellow_lane_detector = DetectLane.YellowLaneDetector()
        self.traffic_sub = Traffic.Traffic()

        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)

        self.steer_msg = Float64()
        self.speed_msg = Float64()

        self.speed_msg.data = 1500 # Default speed
        self.mission_flag = Mission.STRAIGHT.value

        # self.mission_completed = [True, True, True, False, False]  # 각 미션 완료 여부를 저장하는 리스트
        self.mission_completed = [False] * 5  # 각 미션 완료 여부를 저장하는 리스트

        self.rotary_state = 0       # 로터리 내 하위 상태 
        self.rotary_entry_time = 0  # 상태 전환을 위한 타이머

        # 신호등 미션용 상태 변수 및 타이머 추가
        self.traffic_light_state = 0 # 0: 대기, 1: 회전 실행, 2: 자세 교정
        self.traffic_light_timer = 0        

    def action(self, width, height, warped_img, bin_img):
        self.width = width
        self.height = height
        left_fit, right_fit = self.sliding.apply(bin_img)

        self.stop_lane_detector.detect(width,height,bin_img)

        if self.mission_flag == Mission.STRAIGHT.value:
            print("STRAIGHT LINE")
            self.setSteeringinStraight(bin_img)
            if self.stop_lane_detector.stop_line_num == 3:
                self.mission_flag = Mission.FIRST_CORNER.value
            elif self.stop_lane_detector.stop_line_num == 4 and self.mission_completed[2] == True: # 두번째 코너 확인하기 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
                self.mission_flag = Mission.SECOND_CORNER.value
            elif self.stop_lane_detector.stop_line_num == 5:
                self.mission_flag = Mission.ROTARY.value
            elif self.stop_lane_detector.stop_line_num == 9:
                self.mission_flag = Mission.FINISH.value

        elif self.mission_flag == Mission.FIRST_CORNER.value:
            print("FIRST CORNER")
            self.speed_msg.data = 1000
            self.setSteeringinCurve(left_fit, right_fit)

            if left_fit is not None and right_fit is not None:
                self.setSteeringinCurve(left_fit, right_fit)
            else:
                self.setSteeringinStraight(bin_img) # 차선을 놓치면 직진으로 임시 보정
            
            # 코너 탈출 조건: 조향각이 거의 중앙으로 돌아오면 직선 주행으로 복귀
            if abs(self.steer_msg.data - 0.5) < 0.1:
                print("Out from First Corner")
                self.mission_flag = Mission.STRAIGHT.value
        
        elif self.mission_flag == Mission.SECOND_CORNER.value:  # 두번째 코너 확인하기 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
            print("SECOND CORNER")
            try:
                if left_fit is not None and right_fit is not None:
                    self.setSteeringinCurve(left_fit, right_fit)
                else:
                    self.setSteeringinStraight(bin_img)
            except Exception as e:
                pass

        elif self.mission_flag == Mission.ROTARY.value:
            print(f"Enter to Rotary, State: {self.rotary_state}")

            # ----------------------------------------------------
            # 상태 0: 로터리 진입 시 장애물이 있으면 일단 정지
            # ----------------------------------------------------   
            if self.rotary_state == 0:
                pass

            # ----------------------------------------------------
            # 상태 1: 로터리 진입 (장애물이 없을 때, 회전 타이밍 감지하기 위함)
            # ----------------------------------------------------
            elif self.rotary_state == 1:
                self.speed_msg.data = 800  # 1. 속도 감속
                self.steer_msg.data = 0.5
                
                # 타이머 시작 (최초 진입 시 1회만)
                if self.rotary_entry_time == 0:
                    self.rotary_entry_time = time.time()

                # 약 1.5초간 대각선으로 진입 후 다음 상태로 전환
                if self.stop_lane_detector.estimated_distance <= -0.1:
                    self.rotary_state = 2
                    self.rotary_entry_time = 0 # 타이머 초기화

            # ----------------------------------------------------
            # 상태 2: 로터리 횡단 (회전 후 직진하며 노란선 찾기)
            # ----------------------------------------------------
            elif self.rotary_state == 2:
                self.speed_msg.data = 1000 # 횡단 속도
                self.steer_msg.data = 0.77  # 진입 시 보다 좀 더 틀기

                # 3. 노란색 라인 검출
                is_yellow_found = self.yellow_lane_detector.detect(warped_img)

                if is_yellow_found:
                    print("Yellow Lane Detected! Start Exit Sequence.")
                    self.rotary_state = 3
                    self.rotary_entry_time = time.time() # 탈출 타이머 시작

            # ----------------------------------------------------
            # 상태 3: 로터리 탈출 (좀 더 꺾어서 차선 맞추기)
            # ----------------------------------------------------
            elif self.rotary_state == 3:
                self.speed_msg.data = 800  # 탈출 시 감속
                self.steer_msg.data = 0.95 # 4. 차선 복귀를 위해 더 큰 각도로 우회전
                
                # 약 1.0초간 크게 꺾어 차선에 맞춘 후 로터리 미션 완료
                if time.time() - self.rotary_entry_time > 1.0:
                    print("Escape Rotary Complete!")
                    self.mission_flag = Mission.TRAFFICLIGHT.value
                    self.mission_completed[3] = True # 로터리 미션 완료 처리
                    self.rotary_state = 0 # 로터리 상태 초기화

        elif self.mission_flag == Mission.TRAFFICLIGHT.value:
            
            self.traffic_run(bin_img) 

            if self.stop_lane_detector.stop_line_num == 7:
                self.mission_completed[4] = True  # 5번째 미션 완료
                self.mission_flag = Mission.THIRD_CORNER.value  # 세번째 코너로 이동

        elif self.mission_flag == Mission.THIRD_CORNER.value:
            print("THIRD CORNER")
            try:
                if left_fit is not None and right_fit is not None:
                    self.setSteeringinCurve(left_fit, right_fit)
                else:
                    self.setSteeringinStraight(bin_img)
            except Exception as e:
                pass

            if self.stop_lane_detector.stop_line_num == 8:
                self.mission_flag = Mission.STRAIGHT.value  # 세번째 코너 탈출 후 직진으로 복귀

        elif self.mission_flag == Mission.FINISH.value:
            print("FINISH LINE")


        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)


    def traffic_run(self, bin_img):
        signal = self.traffic_sub.traffic_signal
        # print(self.stop_line_distance, "m away")
        
        if self.traffic_light_state == 0:
            self.setSteeringinStraight(bin_img)

            if signal == 1:  # 빨간불
                if self.stop_lane_detector.stop_line_detected and self.stop_lane_detector.stop_line_distance <= 0.3:
                    self.speed_msg.data = 0  # 정지선 0.3m 이내에서만 정지
                elif self.stop_lane_detector.stop_line_detected and self.stop_lane_detector.stop_line_distance <= 1.0:
                    self.speed_msg.data = 300  # 정지선 근처에서 감속
                else:
                    self.speed_msg.data = 500
            elif signal == 4:
                self.speed_msg.data = 300
            elif signal == 16 or signal == 33:
                self.traffic_light_timer = time.time()
                self.traffic_light_state = 1

        elif self.traffic_light_state == 1:
            self.speed_msg.data = 1000
            self.steer_msg.data = 0.5  # 중앙으로 조향
            elapsed_time = time.time() - self.traffic_light_timer
            
            print(f"Executing Timed Turn... Time: {elapsed_time:.1f}s")
            
            # 바깥 차선이면 1.15 안쪽 차선이면 1.0
            if elapsed_time > 1.05: # 1.05는 바깥 차선에서 돌 때 약간 모자라긴 한데 나중에 한 1.07로 테스트 해볼 것
                print("Main turn finished, correcting posture.")
                self.traffic_light_timer = time.time() # 타이머 리셋
                self.traffic_light_state = 2

        elif self.traffic_light_state == 2:
            self.speed_msg.data = 1000
            elapsed_time = time.time() - self.traffic_light_timer
            
            self.steer_msg.data = 0.20 # 좌회전 조향각
            print(f"Executing Timed Turn... Time: {elapsed_time:.1f}s")

            # 2.5초간 회전 후 '자세 교정' 상태로 전환
            if elapsed_time > 2.5:
                print("Main turn finished, correcting posture.")
                self.traffic_light_timer = time.time()
                self.traffic_light_state = 3 # ⭐️ '자세 교정' 상태로 전환

        elif self.traffic_light_state == 3:
            self.speed_msg.data = 800 # 교정 시 속도
            self.setSteeringinStraight(bin_img)

            elapsed_time = time.time() - self.traffic_light_timer
            
            # ⭐️ 1.5초 동안 반대(0.7) 조향으로 자세를 바로잡음 (튜닝 필요)
            print(f"Correcting Posture... Time: {elapsed_time:.1f}s")

            # 1.5초가 지나면 모든 과정을 완료
            if elapsed_time > 1.5:
                print("Posture corrected. Traffic light mission complete.")
                # 미션 완료 처리 및 상태 초기화
                self.mission_completed[4] = True
                self.mission_flag = Mission.THIRD_CORNER.value
                self.traffic_light_state = 0


    def setSteeringinStraight(self, bin_img):
        """
        [최종 개선 버전] 한쪽 차선만 보일 때도 가상 중앙선을 추정하여 주행 안정성을 높인 함수
        """
        histogram = np.sum(bin_img[self.height // 2:, :], axis=0)
        midpoint = self.width // 2

        # 1. 히스토그램을 좌우로 나누고 각 영역의 차선 피크(Peak)를 찾음
        left_half = histogram[:midpoint]
        right_half = histogram[midpoint:]
        leftx_base = np.argmax(left_half)
        rightx_base = np.argmax(right_half) + midpoint

        # 이상적인 차선 폭 (전체 이미지 폭의 70%로 가정, 필요시 튜닝)
        LANE_WIDTH_PIXELS = self.width * 0.7

        # --- 차선 검출 상태에 따른 중앙점 계산 ---
        
        # CASE 1: 양쪽 차선이 모두 잘 보일 때 (가장 이상적인 경우)
        if histogram[leftx_base] > 50 and histogram[rightx_base] > 50:
            lane_center = (leftx_base + rightx_base) / 2
            # print("Status: Two lanes detected (1)")
        
        # CASE 2: 한쪽 차선만 보일 때 (핵심 개선 부분)
        else:
            # 오른쪽 차선만 보일 경우
            if histogram[rightx_base] > 50:
                # 오른쪽 차선 위치에서 차선 폭의 절반만큼 왼쪽으로 이동한 지점을 가상 중앙으로 설정
                lane_center = rightx_base - (LANE_WIDTH_PIXELS / 2)
                # print("Status: Right lane only (2-Right)")
            # 왼쪽 차선만 보일 경우
            elif histogram[leftx_base] > 50:
                # 왼쪽 차선 위치에서 차선 폭의 절반만큼 오른쪽으로 이동한 지점을 가상 중앙으로 설정
                lane_center = leftx_base + (LANE_WIDTH_PIXELS / 2)
                # print("Status: Left lane only (2-Left)")
            # CASE 3: 양쪽 차선이 모두 안 보일 경우
            else:
                lane_center = midpoint
                # print("Status: No lanes detected (3)")
        
        # 최종 조향각 계산
        offset = (lane_center - midpoint) / midpoint
        steer = 0.5 + offset * 0.7
        self.steer_msg.data = np.clip(steer, 0.0, 1.0)

    def setSteeringinCurve(self, left_fit, right_fit):
        y_eval = int(self.height * 0.6)
        left_x = np.polyval(left_fit, y_eval)
        right_x = np.polyval(right_fit, y_eval)
        lane_center = (left_x + right_x) / 2
        img_center = self.width / 2
        offset = (lane_center - img_center) / img_center

        a = (left_fit[0] + right_fit[0]) / 2.0  # 곡률 정보
        curvature_threshold = 5e-4
        steer_base = 0.5 + offset * 0.8

        if abs(a) > curvature_threshold:
            steer_base += np.sign(offset) * abs(a) * 18

        self.steer_msg.data = np.clip(steer_base, 0.0, 1.0)

    def setSteeringForInnerRightTurn(self, right_fit):
        """
        [신규] 안쪽(오른쪽) 차선을 기준으로 일정한 거리를 유지하며 우회전하는 함수
        """
        # 튜닝 포인트: 차량 중심이 안쪽 차선에서 떨어져야 할 이상적인 거리 (픽셀 단위)
        # 이 값을 조절하여 회전 반경을 조절할 수 있습니다.
        DESIRED_OFFSET_PIXELS = 120  # 예시 값, 주행해보며 튜닝 필요

        # 차선 인식을 위한 기준 높이 (화면의 70% 지점)
        y_eval = int(self.height * 0.7)
        
        # 해당 높이에서 오른쪽 차선의 x좌표 계산
        right_x = np.polyval(right_fit, y_eval)
        
        # 목표 주행 지점 설정: 오른쪽 차선 위치 - 이상적인 오프셋
        target_x = right_x - DESIRED_OFFSET_PIXELS
        
        # 차량의 현재 중심(이미지 중앙)
        img_center = self.width / 2
        
        # 목표 지점과 현재 중심 사이의 오차 계산
        offset = target_x - img_center

        # 오차를 기반으로 조향각 계산 (게인 값 0.9로 약간 더 민감하게 설정)
        steer = 0.5 + (offset / img_center) * 0.9
        
        self.steer_msg.data = np.clip(steer, 0.0, 1.0)
        print(f"Inner Right Turn... Target X: {target_x:.1f}, Steer: {self.steer_msg.data:.2f}")

