#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from morai_msgs.msg import GetTrafficLightStatus
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
from time import *

class Traffic_Control:
    def __init__(self):
        rospy.init_node("lane_sub_node")
        self.steer_pub = rospy.Publisher("/commands/servo/position",Float64,queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1)

        rospy.Subscriber("/GetTrafficLightStatus",GetTrafficLightStatus,self.traffic_CB)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        
        self.bridge = CvBridge()
        self.steer_msg = Float64()
        self.speed_msg = Float64()
        self.traffic_msg = GetTrafficLightStatus()
        self.traffic_flag = 0
        self.prev_signal = 0
        self.signal = 0
        self.cross_flag = 0
        self.img = 0
        self.x = 0
        self.y = 0

    def traffic_CB(self,msg):
        self.traffic_msg = msg

        if self.traffic_msg.trafficLightIndex == "SN00002":
            self.signal=self.traffic_msg.trafficLightStatus
            if self.prev_signal != self.signal:
                self.prev_signal = self.signal
            self.traffic_think()

    def traffic_think(self):
        if self.signal == 1 :
            print("red")
        elif self.signal == 4:
            print("yellow")
        elif self.signal == 16:
            print("green")
        elif self.signal == 33:
            print("left")
        else:
            pass
    
    def cam_CB(self,msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.cam_lane_detection()

    def cam_lane_detection(self):
        img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(img_hsv)
        y,x = self.img.shape[0:2]
        # cv2.imshow("h",h)
        # cv2.imshow("s",s)
        # cv2.imshow("v",v)

        yellow_lower = np.array([15,128,0])  
        yellow_upper = np.array([45,255,255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        white_lower = np.array([0,0,192])  
        white_upper = np.array([179,64,255])
        white_range = cv2.inRange(img_hsv, white_lower, white_upper)

        combined_range = cv2.bitwise_or(yellow_range, white_range)
        filtered_range = cv2.bitwise_and(self.img,self.img,mask = combined_range)
        
        src_point1 = [0,420]
        src_point2 = [275,260]
        src_point3 = [x-275,260]
        src_point4 = [x,420]
        src_points = np.float32([src_point1,src_point2,src_point3,src_point4])
        
        dst_point1 = [x//8,480]
        dst_point2 = [x//8,0]
        dst_point3 = [x//8*7,0]
        dst_point4 = [x//8*7,480]
        dst_points = np.float32([dst_point1,dst_point2,dst_point3,dst_point4])

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filtered_range, matrix, [x,y])

        grayed_img = cv2.cvtColor(warped_img,cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(grayed_img)
        bin_img[grayed_img>50]=255
        histogram = np.sum(bin_img, axis=0)
        left_hist = histogram[0:x//2]
        right_hist = histogram[x//2:]
        
        left_indices = np.where(left_hist>20)[0]
        right_indices = np.where(right_hist>20)[0]+320
        indices = np.where(histogram>20)[0]

        try:
            if len(left_indices) != 0 and len(right_indices) == 0:
                center_idx = (left_indices[0]+right_indices[-1])//2
            elif len(left_indices) == 0 and len(right_indices) != 0:
                center_idx = (right_indices[0] + left_indices[-1])//2
            else:
                center_idx = (indices[0] + indices[-1])//2
        except:
            center_idx = x//2
        canny_img = cv2.Canny(bin_img,2,2)
        
        lines = cv2.HoughLinesP(canny_img, 0.01, np.pi/180, 90, 100, 450)
        try:
            for line in lines:
                x1,y1,x2,y2 = line[0]
                cv2.line(warped_img, (x1,y1), (x2,y2), (0,255,0), 1)
        except:
            pass
        
        standard_line = x//2
        degree_per_pixel = 1/x

    def action(self):
        if self.img != 0:
            steer = (center_idx - standard_line) * degree_per_pixel
            steer += 0.5

            self.steer_msg.data = steer
            self.speed_msg.data = 1000
            self.speed_pub.publish(self.speed_msg)
            self.steer_pub.publish(self.steer_msg)

            cv2.imshow("img",self.img)
            cv2.imshow("warped_img",warped_img)
            cv2.imshow("bin_img",bin_img)
            
            cv2.waitKey(1)
    

def main():
    try:
        traffic_control = Traffic_Control()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()