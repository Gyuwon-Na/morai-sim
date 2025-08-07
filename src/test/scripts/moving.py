#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

class Lane_sub:
    def __init__(self):
        rospy.init_node("lane_steering_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)

        self.bridge = CvBridge()
        self.steer_msg = Float64()
        self.speed_msg = Float64()
        self.speed_msg.data = 1000  # 기본 속도

    def cam_CB(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.height, self.width = img.shape[:2]

        # 차선 색상 범위(노란색, 흰색)
        yellow_lower = np.array([15, 128, 0])
        yellow_upper = np.array([45, 255, 255])
        white_lower = np.array([0, 0, 192])
        white_upper = np.array([179, 64, 255])

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        filtered = cv2.bitwise_and(img, img, mask=combined_mask)

        # 투시 변환
        src = np.float32([
            [0, 420],
            [275, 260],
            [self.width-275, 260],
            [self.width, 420]
        ])
        dst = np.float32([
            [self.width//8, 480],
            [self.width//8, 0],
            [self.width//8*7, 0],
            [self.width//8*7, 480]
        ])
        M = cv2.getPerspectiveTransform(src, dst)
        warped_img = cv2.warpPerspective(filtered, M, (self.width, self.height))

        # 이진화
        gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(gray)
        bin_img[gray > 50] = 255

        # 선 검출
        edges = cv2.Canny(bin_img, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=20, maxLineGap=15)

        warped_color = warped_img.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(warped_color, (x1, y1), (x2, y2), (0, 255, 0), 2)

        self.setCenter(bin_img)

        # 시각화
        cv2.imshow("Warped + Lines", warped_color)
        cv2.waitKey(1)


    def setCenter(self, bin_img):
        # 차선 중심 계산 (histogram 방식)
        histogram = np.sum(bin_img[self.height//2:, :], axis=0)
        midpoint = self.width // 2
        lane_center = np.mean(np.where(histogram > 50)) if np.any(histogram > 50) else midpoint

        # 조향 계산
        is_corner = self.isCorner()

        if is_corner is False:
            offset = (lane_center - midpoint) / midpoint
            steer = 0.5 + offset * 0.4                   
            steer = np.clip(steer, 0.0, 1.0)
        else:
            steer = 0.5

        # 메시지 퍼블리시
        self.steer_msg.data = steer
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)
        
    def isCorner(self):
        return False

def main():
    try:
        Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
