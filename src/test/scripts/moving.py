#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class LaneFollower:
    def __init__(self):
        rospy.init_node("lane_follower_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_callback)

        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)

        self.bridge = CvBridge()
        self.steer_msg = Float64()
        self.speed_msg = Float64()
        self.speed_msg.data = 1000  # 기본 속도

    def cam_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, w = img.shape[:2]

        # 색상 필터링 (노란선 + 흰선)
        yellow_lower = np.array([15, 128, 0])
        yellow_upper = np.array([45, 255, 255])
        white_lower = np.array([0, 0, 192])
        white_upper = np.array([179, 64, 255])

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

        # 투시 변환
        src = np.float32([[0, 420], [275, 260], [w - 275, 260], [w, 420]])
        dst = np.float32([[w // 8, 480], [w // 8, 0], [w // 8 * 7, 0], [w // 8 * 7, 480]])
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(combined_mask, M, (w, h))

        # 차선 중심 계산
        histogram = np.sum(warped[warped.shape[0]//2:, :], axis=0)
        midpoint = w // 2
        lane_center = np.mean(np.where(histogram > 50)) if np.any(histogram > 50) else midpoint

        # 조향 계산
        offset = (lane_center - midpoint) / midpoint  # 정규화 (-1 ~ 1)
        steer = 0.5 + offset * 0.4                    # 조향 감도 조절
        steer = np.clip(steer, 0.0, 1.0)

        # 메시지 발행
        self.steer_msg.data = steer
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)

        # 디버그 출력
        rospy.loginfo(f"[조향] offset: {offset:.2f}, steer: {steer:.2f}")

        # 시각화
        warped_bgr = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        cv2.line(warped_bgr, (int(lane_center), h), (int(lane_center), h - 50), (0, 255, 0), 3)
        cv2.line(warped_bgr, (midpoint, h), (midpoint, h - 50), (255, 0, 0), 3)
        cv2.imshow("Warped", warped_bgr)
        cv2.waitKey(1)

def main():
    try:
        LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()

